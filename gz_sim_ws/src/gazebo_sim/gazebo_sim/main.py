#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
from myQueue import queue
import time

class gazebo_node(Node): # Instância da classe Node

    def __init__(self):
        super().__init__('gazebo_controller')
        self.get_logger().info("Inicializando o nó gazebo_controller...")

        self.fila = queue([])  # Cria uma instância da classe "queue" (fila)

        # Cria um assinante para o tópico "/odom" e define o callback "odometry_callback" para lidar com as mensagens recebidas
        self.odometry_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.odometry_callback,
            qos_profile=10
        )

        # Cria um publicador para o tópico "/cmd_vel" para enviar comandos de velocidade
        self.velocity_publisher = self.create_publisher(
            msg_type=Twist,
            topic='/cmd_vel',
            qos_profile=10
        )

        self.pose = [0, 0, 0]  # Variável para armazenar a pose atual do robô [x, y, theta]
        self.relativePose = [0, 0]  # Variável para armazenar a pose relativa em relação ao objetivo [distância, ângulo]
        self.isMoving = False  # Indica se o robô está em movimento
        self.minDistance = 10000  # Variável para armazenar a menor distância até o objetivo (inicializada com um valor alto para garantir que a primeira distância seja menor)

    def rota_padrão(self):
        # Menu simples para escolher entre a rota padrão ou personalizada
        print("Pressione 1 para executar a rota padrão ou 2 para personalizar uma rota!")
        opcao = input()
        while opcao != "1" and opcao != "2":
            print("Opção inválida, digite novamente!")
            opcao = input()
        if opcao == "1":
            self.fila.queue([3,4])  # Adiciona o ponto pontos à fila
            self.fila.queue([2,-5])
            self.fila.queue([5,-5])
        else:
            print("Digite a quantidade de pontos que deseja percorrer: ")
            qtd = int(input())
            for i in range(qtd):
                print("Digite o ponto que deseja percorrer: ")
                print("x: ")
                x = int(input())
                print("y: ")
                y = int(input())
                self.fila.queue([x,y])  # Adiciona pontos [x, y] à fila
        self.fila.showQueue()  # Exibe o conteúdo atual da fila

    def odometry_callback(self, msg):
        # Transforma a orientação do robô de quaternion para ângulo
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        pose = [round(x, 2), round(y, 2), round(theta, 2)]
        print("Posição atual:", pose)
        self.pose = pose
        if self.fila.isEmpty(): # Verifica se a fila está vazia
            self.get_logger().info("Fila vazia, pressione Ctrl+C para sair")
        else: # Caso contrário, chama a função "calculation"
            self.calculation(self.fila.first())

    # Função para calcular a distância e o ângulo entre o robô e o objetivo
    def calculation(self, track):
        if self.isMoving == True:
            # Se o robê estiver em movimento, apenas atualiza a distância
            distance = math.sqrt(((track[0] - self.pose[0]) ** 2) + ((track[1] - self.pose[1]) ** 2))
            self.relativePose[0] = round(distance,2)
            print(self.relativePose)
        else:
            # Calcula a distância entre o robô e o objetivo
            distance = math.sqrt(((track[0] - self.pose[0]) ** 2) + ((track[1] - self.pose[1]) ** 2))
            # Calcula o ângulo entre o robô e o objetivo
            angle = math.atan2(track[1] - self.pose[1], track[0] - self.pose[0])
            # Calcula a diferença de ângulo entre o robô e o objetivo
            angle_difference = angle - self.pose[2]
            # Retorna a distância e a diferença de ângulo
            print("Distância linear/angular até o objetivo:", [round(distance,2), round(angle_difference, 3)])
            self.relativePose[0] = round(distance,2)
            self.relativePose[1] = round(angle_difference, 3)
        # Verifica se relativePose[0] é menor que minDistance com uma tolerância de 0.05 (inércia), e assim atualiza a menor distância
        if round(self.relativePose[0],1) < round(self.minDistance,1) + 0.05:
            self.minDistance = self.relativePose[0]
            print("Menor distância:", self.minDistance, "menor distância relativa:" , self.relativePose[0])
        self.move() # Chama a função "move"

    def move(self):
        # Verifica se o robô está próximo o suficiente do objetivo, ou se está se distanciando
        if self.relativePose[0] > 0.1 and self.minDistance == self.relativePose[0]:
            # Verifica se o ângulo está próximo de zero, ou seja, se o robô está alinhado com o objetivo
            if self.relativePose[1] < 0.01 and self.relativePose[1] > -0.01:
                # Cria uma mensagem Twist para mover o robô linearmente
                self.isMoving = True
                self.minDistance = self.relativePose[0]
                vel_msg = Twist()
                vel_msg.linear.x = 1.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
            # Caso contrário, gira o robô para alinhar com o objetivo    
            else:
                # Cria uma mensagem Twist para girar o robô, a fim de ajustar o ângulo
                print("Rotacionando para encontrar a melhor rota...")
                vel_msg = Twist()
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.1
                self.velocity_publisher.publish(vel_msg)
        # Se o robô já tiver feito a trajetória, reseta as variáveis e parte para o próximo objetivo
        else:
            self.isMoving = False
            self.minDistance = 10000
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
            self.fila.dequeue()  # Remove o primeiro elemento da fila
            self.fila.showQueue()  # Exibe o conteúdo atual da fila
            time.sleep(3)  # Aguarda 3 segundos para o robô parar completamente antes de calcular a distância novamente

def main(args=None):
    rclpy.init(args=args)
    node = gazebo_node()
    node.rota_padrão()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
