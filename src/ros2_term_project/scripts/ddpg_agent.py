#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense, Lambda, concatenate
from tensorflow.keras.optimizers import Adam
import tensorflow as tf
import numpy as np
from collections import deque
import random
import os
import signal
import sys
import time
import datetime
from robot_state_msgs.msg import RobotState

class ReplayBuffer:
    def __init__(self, buffer_size):
        self.buffer_size = buffer_size
        self.buffer = deque(maxlen=buffer_size)

    def add(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        if len(self.buffer) < batch_size:
            return None
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return (np.array(states), 
                np.array(actions), 
                np.array(rewards), 
                np.array(next_states), 
                np.array(dones))

    def __len__(self):
        return len(self.buffer)

class Actor(Model):
    def __init__(self, state_dim, action_dim, action_bound):
        super().__init__()
        self.action_bound = action_bound
        
        # 네트워크 구조 개선
        self.fc1 = Dense(256, activation='relu')
        self.fc2 = Dense(128, activation='relu')
        self.fc3 = Dense(64, activation='relu')
        self.action = Dense(action_dim, activation='tanh')

    def call(self, state):
        x = self.fc1(state)
        x = self.fc2(x)
        x = self.fc3(x)
        action = self.action(x)
        
        # 액션 스케일링
        scaled_action = Lambda(lambda x: x * self.action_bound)(action)
        return scaled_action

class Critic(Model):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        
        # 네트워크 구조 개선
        self.state_fc = Dense(256, activation='relu')  # state 처리를 위한 레이어 추가
        self.fc1 = Dense(256, activation='relu')
        self.fc2 = Dense(128, activation='relu')
        self.fc3 = Dense(64, activation='relu')
        self.q_value = Dense(1, activation='linear')

    def call(self, inputs):
        state, action = inputs
        x = self.fc1(action)  # [batch_size, 256]
        state_processed = self.state_fc(state)  # [batch_size, 256]
        x = concatenate([state_processed, x])
        x = self.fc2(x)
        x = self.fc3(x)
        q_value = self.q_value(x)
        return q_value
class OUNoise:
    """Ornstein-Uhlenbeck process noise generator"""
    def __init__(self, action_dim, mu=0, theta=0.15, sigma=0.2):
        self.action_dim = action_dim
        self.mu = mu * np.ones(action_dim)
        self.theta = theta
        self.sigma = sigma
        self.state = np.ones(action_dim)
        self.reset()

    def reset(self):
        """Reset the noise state"""
        self.state = np.copy(self.mu)

    def noise(self, decay_factor=1.0):
        """Generate noise"""
        dx = self.theta * (self.mu - self.state) + \
             self.sigma * decay_factor * np.random.randn(len(self.state))
        self.state += dx
        return self.state

class DDPGAgent(Node):
    def __init__(self):
        super().__init__('ddpg_agent')
        self.get_logger().info('Starting DDPG Agent initialization...')
        # 로깅 설정
        self.log_file = open('/home/jewoo/ros2_ws/src/ros2_term_project/logs/ddpg_agent_log.txt', 'w')
        self.log_frequency = 10
        self.log_counter = 0

        # Tensorboard 설정 추가
        current_time = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        self.train_log_dir = '/home/jewoo/ros2_ws/src/ros2_term_project/logs/tensorboard/' + current_time
        self.summary_writer = tf.summary.create_file_writer(self.train_log_dir)

        # Parameter 선언
        self.declare_parameter('is_training', True)  # 기본값은 True
        self.is_training = self.get_parameter('is_training').value
        
        self.get_logger().info(f'Training mode: {self.is_training}')
        self.write_log(f'Training mode: {self.is_training}')

        # 학습 추적 변수
        self.episode_count = 0
        self.total_rewards = []
        self.last_log_time = time.time()

        # TensorFlow 로깅 비활성화
        tf.get_logger().setLevel('ERROR')

        # 상태 차원 초기화
        self.local_map_size = None
        self.STATE_DIM = None
        
        # 액션 설정
        self.ACTION_DIM = 2  # 선형, 각속도
        self.ACTION_BOUND = np.array([0.1, 0.2])  # 최대 속도 조정
        
        # DDPG 하이퍼파라미터
        self.GAMMA = 0.99
        self.TAU = 0.001
        self.BATCH_SIZE = 64
        self.BUFFER_SIZE = 100000
        self.NOISE_SCALE = 0.2  # 탐험을 위한 노이즈 증가
        
        # Optimizer 설정
        self.actor_optimizer = Adam(learning_rate=0.001)
        self.critic_optimizer = Adam(learning_rate=0.002)

        # 네트워크 초기화 지연
        self.actor = None
        self.critic = None
        self.target_actor = None
        self.target_critic = None

        # 상태 변수
        self.current_state = None
        self.last_action = None
        self.last_reward = 0.0

        # ROS 통신 설정
        self.replay_buffer = ReplayBuffer(self.BUFFER_SIZE)
        
        # 로봇 상태 구독자
        self.robot_state_sub = self.create_subscription(
            RobotState,
            'robot_state',
            self.robot_state_callback,
            10
        )

        # 보상 구독자
        self.reward_sub = self.create_subscription(
            Float32,
            'reward',
            self.reward_callback,
            10
        )

        # 속도 발행자
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # OU 노이즈 초기화
        self.noise = OUNoise(
            action_dim=self.ACTION_DIM,
            mu=0.0,       # 평균값
            theta=0.15,   # 평균 회귀 속도
            sigma=self.NOISE_SCALE  # 노이즈 크기
        )
        self.noise_decay = 1.0
        self.noise_decay_rate = 0.995  # 매 에피소드마다 0.5% 감소
        self.min_noise_decay = 0.1     # 최소 10%까지만 감소

        # 에피소드 종료 구독
        self.episode_end_sub = self.create_subscription(
            Bool, 
            'episode_end',
            self.episode_end_callback,
            10
        )

        self.get_logger().info('DDPG 에이전트 초기화 완료')
        self.write_log('DDPG 에이전트 초기화 완료')
        self.get_logger().info('DDPG Agent initialization completed')



    def episode_end_callback(self, msg):
        """에피소드가 종료될 때마다 호출되는 콜백"""
        if msg.data:
            self.episode_count += 1
            # 노이즈 감쇄
            self.noise_decay = max(
                self.min_noise_decay,
                self.noise_decay * self.noise_decay_rate
            )
            # 노이즈 상태 리셋
            self.noise.reset()
            self.get_logger().info(f'Episode {self.episode_count} ended. ' + 
                                 f'Current noise decay: {self.noise_decay:.3f}')

    def write_log(self, message):
        """로그 파일에 메시지 기록"""
        current_time = time.strftime('%Y-%m-%d %H:%M:%S')
        log_entry = f"[{current_time}] {message}\n"
        self.log_file.write(log_entry)
        self.log_file.flush()
        print(log_entry.strip())  # 터미널에도 출력

    def robot_state_callback(self, msg):
        """로봇 상태 콜백"""
        try:
            # 첫 수신 시 상태 차원 설정
            if self.local_map_size is None:
                self.local_map_size = len(msg.local_map)
                local_map_dim = int(np.sqrt(self.local_map_size))
                
                # 상태 차원 계산 
                self.STATE_DIM = (
                    self.local_map_size +  # 로컬 맵
                    3 +  # x, y, yaw
                    2 +  # 선형/각속도 
                    2    # known_cells, frontier_cells
                )
                
                # 네트워크 초기화
                self.actor = Actor(self.STATE_DIM, self.ACTION_DIM, self.ACTION_BOUND)
                self.critic = Critic(self.STATE_DIM, self.ACTION_DIM)
                self.target_actor = Actor(self.STATE_DIM, self.ACTION_DIM, self.ACTION_BOUND)
                self.target_critic = Critic(self.STATE_DIM, self.ACTION_DIM)
                
                # 옵티마이저 설정
                self.actor.compile(optimizer=self.actor_optimizer)
                self.critic.compile(optimizer=self.critic_optimizer)
                
                # 초기 가중치 복사
                self.target_actor.set_weights(self.actor.get_weights())
                self.target_critic.set_weights(self.critic.get_weights())
                
                # 저장된 모델 불러오기
                self.load_model()
                
                self.write_log(f'네트워크 초기화 완료. 상태 차원: {self.STATE_DIM}')

            # 상태 벡터 구성
            local_map = np.array(msg.local_map, dtype=np.float32) / 100.0  # 정규화
            state = np.concatenate([
                local_map,  # 로컬 맵
                [msg.x, msg.y, msg.yaw],  # 로봇 위치
                [msg.linear_velocity, msg.angular_velocity],  # 속도
                [msg.known_cells_count, msg.frontier_cells_count]  # 탐험 메트릭
            ])
            
            self.current_state = state
            
             # 액션 선택 및 발행
            if self.actor is not None:
                action = self.select_action(state)
                cmd_vel = Twist()
                cmd_vel.linear.x = float(action[0])
                cmd_vel.angular.z = float(action[1])
                
                # cmd_vel 발행 전 로깅 추가
                if self.log_counter % self.log_frequency == 0:
                    self.write_log(f'Publishing cmd_vel - linear: {cmd_vel.linear.x:.3f}, angular: {cmd_vel.angular.z:.3f}')
                    self.write_log(f'Robot State - x: {msg.x:.2f}, y: {msg.y:.2f}, yaw: {msg.yaw:.2f}')
                    # Tensorboard에 발행된 속도 기록
                    with self.summary_writer.as_default():
                        tf.summary.scalar('cmd_vel/linear', cmd_vel.linear.x, step=self.log_counter)
                        tf.summary.scalar('cmd_vel/angular', cmd_vel.angular.z, step=self.log_counter)
                
                self.cmd_vel_pub.publish(cmd_vel)
                self.last_action = action

        except Exception as e:
            self.write_log(f'로봇 상태 콜백 오류: {str(e)}')

 
            
        
    def select_action(self, state):
        """액션 선택"""
        try:
            state_tensor = tf.convert_to_tensor([state], dtype=tf.float32)
            action = self.actor(state_tensor).numpy()[0]
            
            # 학습 중에만 노이즈 추가
            if self.is_training:
                noise = self.noise.noise(self.noise_decay)
                action = np.clip(action + noise, -self.ACTION_BOUND, self.ACTION_BOUND)
                
                if self.log_counter % self.log_frequency == 0:
                    self.write_log(f'Training mode - Action with noise: {action}')

                    # Tensorboard에 액션 값 기록
                    with self.summary_writer.as_default():
                        tf.summary.scalar('action/linear_velocity', action[0], step=self.log_counter)
                        tf.summary.scalar('action/angular_velocity', action[1], step=self.log_counter)
                        tf.summary.scalar('action/noise_linear', noise[0], step=self.log_counter)
                        tf.summary.scalar('action/noise_angular', noise[1], step=self.log_counter)
                        tf.summary.scalar('action/noise_decay', self.noise_decay, step=self.log_counter)
            else:
                action = np.clip(action, -self.ACTION_BOUND, self.ACTION_BOUND)
                if self.log_counter % self.log_frequency == 0:
                    self.write_log(f'Evaluation mode - Action without noise: {action}')
                    
            return action
        except Exception as e:
            self.write_log(f'액션 선택 오류: {str(e)}')
        
    def reward_callback(self, msg):
        """보상 콜백"""
        try:
            self.last_reward = msg.data
            
            # 로그 및 디버그
            self.log_counter += 1
            if self.log_counter % self.log_frequency == 0:
                self.write_log(f'보상 수신: {msg.data}, 버퍼 크기: {len(self.replay_buffer)}')
                
                # Tensorboard에 현재 reward 기록
                with self.summary_writer.as_default():
                    tf.summary.scalar('rewards/current_reward', msg.data, step=self.log_counter)
            
            # 충분한 상태와 액션이 있는 경우에만 처리
            if (self.current_state is not None and 
                self.last_action is not None and 
                self.actor is not None):
                
                # 다음 상태는 현재 상태로 (환경에서 상태 업데이트 대기)
                next_state = self.current_state
                
                # 리플레이 버퍼에 추가
                self.replay_buffer.add(
                    self.current_state,  # 현재 상태
                    self.last_action,    # 마지막 액션
                    msg.data,            # 보상
                    next_state,          # 다음 상태
                    False                # 종료 상태
                )
                
                # 학습 수행
                if len(self.replay_buffer) >= self.BATCH_SIZE:
                    self.train()
                    
                    # 에피소드 추적
                    self.total_rewards.append(msg.data)
                    if len(self.total_rewards) % 10 == 0:
                        avg_reward = np.mean(self.total_rewards[-10:])
                        self.write_log(f'최근 10 에피소드 평균 보상: {avg_reward}')
            # 1000스텝마다 모델 저장
            if self.log_counter % 1000 == 0:
                self.save_model()

        except Exception as e:
            self.write_log(f'보상 콜백 오류: {str(e)}')

    def train(self):
        """DDPG 학습"""
        try:
            # 배치 샘플링
            batch = self.replay_buffer.sample(self.BATCH_SIZE)
            if batch is None:
                return

            states, actions, rewards, next_states, dones = batch
            
            # 텐서 변환
            states = tf.convert_to_tensor(states, dtype=tf.float32)
            actions = tf.convert_to_tensor(actions, dtype=tf.float32)
            rewards = tf.convert_to_tensor(rewards, dtype=tf.float32)
            next_states = tf.convert_to_tensor(next_states, dtype=tf.float32)
            dones = tf.convert_to_tensor(dones, dtype=tf.float32)
            
            # 크리틱 학습
            with tf.GradientTape() as tape:
                target_actions = self.target_actor(next_states)
                target_q = self.target_critic([next_states, target_actions])
                q_target = rewards + self.GAMMA * target_q * (1 - dones)
                current_q = self.critic([states, actions])
                critic_loss = tf.reduce_mean(tf.square(q_target - current_q))
                
            # Tensorboard에 critic loss 기록
            with self.summary_writer.as_default():
                tf.summary.scalar('critic_loss', critic_loss, step=self.log_counter)
                
            # 크리틱 그래디언트 적용
            critic_grads = tape.gradient(critic_loss, self.critic.trainable_variables)
            self.critic_optimizer.apply_gradients(
                zip(critic_grads, self.critic.trainable_variables)
            )
            
            # 액터 학습
            with tf.GradientTape() as tape:
                actions = self.actor(states)
                actor_loss = -tf.reduce_mean(self.critic([states, actions]))
                
            # Tensorboard에 actor loss 기록
            with self.summary_writer.as_default():
                tf.summary.scalar('actor_loss', actor_loss, step=self.log_counter)
                
            # 액터 그래디언트 적용
            actor_grads = tape.gradient(actor_loss, self.actor.trainable_variables)
            self.actor_optimizer.apply_gradients(
                zip(actor_grads, self.actor.trainable_variables)
            )
            
            # 타겟 네트워크 소프트 업데이트
            for target_var, var in zip(self.target_actor.variables, self.actor.variables):
                target_var.assign(self.TAU * var + (1 - self.TAU) * target_var)
            
            for target_var, var in zip(self.target_critic.variables, self.critic.variables):
                target_var.assign(self.TAU * var + (1 - self.TAU) * target_var)

        except Exception as e:
            self.write_log(f'학습 중 오류: {str(e)}')

    def __del__(self):
        """리소스 정리"""
        if hasattr(self, 'log_file'):
            self.log_file.close()
        if hasattr(self, 'summary_writer'):
            self.summary_writer.close()

    def save_model(self):
        try:
            # 모델 저장 경로 설정
            model_dir = '/home/jewoo/ros2_ws/src/ros2_term_project/saved_models'
            os.makedirs(model_dir, exist_ok=True)
            
            actor_path = os.path.join(model_dir, 'actor.weights.h5')
            critic_path = os.path.join(model_dir, 'critic.weights.h5')
            
            # 모델 가중치 저장
            self.actor.save_weights(actor_path)
            self.critic.save_weights(critic_path)
            
            # 저장 성공 여부 확인
            if os.path.exists(actor_path) and os.path.exists(critic_path):
                self.write_log('Models successfully saved')
            else:
                self.write_log('Model files not found after save attempt')
                
        except Exception as e:
            self.write_log(f'Error saving models: {str(e)}')

    def load_model(self):
        try:
            model_dir = '/home/jewoo/ros2_ws/src/ros2_term_project/saved_models'
            actor_path = os.path.join(model_dir, 'actor.weights.h5')
            critic_path = os.path.join(model_dir, 'critic.weights.h5')
            
            if os.path.exists(actor_path) and os.path.exists(critic_path):
                # 저장된 모델이 있으면 로드
                self.actor.load_weights(actor_path)
                self.critic.load_weights(critic_path)
                self.target_actor.load_weights(actor_path)
                self.target_critic.load_weights(critic_path)
                self.write_log('Loaded pre-trained models')
            else:
                # 저장된 모델이 없으면 랜덤 초기화
                self.write_log('No pre-trained models found. Initializing networks with random weights.')
                
                # 더미 데이터로 모델 빌드
                dummy_state = np.zeros((1, self.STATE_DIM))
                dummy_action = np.zeros((1, self.ACTION_DIM))
                
                # Actor 네트워크 초기화
                self.actor(dummy_state)
                self.target_actor(dummy_state)
                
                # Critic 네트워크 초기화
                self.critic([dummy_state, dummy_action])
                self.target_critic([dummy_state, dummy_action])
                
                # 타겟 네트워크에 가중치 복사
                self.target_actor.set_weights(self.actor.get_weights())
                self.target_critic.set_weights(self.critic.get_weights())
                
        except Exception as e:
            self.write_log(f'Error in load_model: {str(e)}')
            # 에러 발생 시에도 네트워크 초기화
            dummy_state = np.zeros((1, self.STATE_DIM))
            dummy_action = np.zeros((1, self.ACTION_DIM))
            
            self.actor(dummy_state)
            self.target_actor(dummy_state)
            self.critic([dummy_state, dummy_action])
            self.target_critic([dummy_state, dummy_action])
            
            self.target_actor.set_weights(self.actor.get_weights())
            self.target_critic.set_weights(self.critic.get_weights())
def main(args=None):
    rclpy.init(args=args)
    agent = None
    try:
        print("Starting DDPG agent...")
        agent = DDPGAgent()
        print("DDPG agent initialized, starting spin...")
        rclpy.spin(agent)
    except Exception as e:
        print(f"Critical error occurred: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        if agent:
            print("Shutting down agent...")
            agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()