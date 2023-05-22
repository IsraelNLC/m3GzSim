# Gazebo Simulation

Nó de Controle do Gazebo

O nó implementado é responsável por controlar o movimento de um robô no ambiente Gazebo. Ele recebe a posição atual do robô do tópico '/odom' e calcula a distância e a diferença de ângulo entre o robô e o objetivo. Com base nesses cálculos, determina se o robô deve se mover ou girar. O objetivo pode ser um trajeto padrão pré-definido ou um trajeto personalizado.

Dependências:

• rclpy

• tf_transformations

Classes:

gazebo_node: Classe principal que implementa o nó de controle do Gazebo.
queue: Classe que implementa uma fila simples.

Funções:

• rota_padrão(): Solicita ao usuário escolher um trajeto padrão ou personalizado e preenche a fila de objetivos.

• odometry_callback(msg): Função de retorno de chamada para o tópico '/odom'. Atualiza a posição atual do robô e chama o cálculo.

• calculation(track): Calcula a distância e a diferença de ângulo entre o robô e o objetivo atual, além de chamar a função move.

• move(): Controla o movimento do robô com base nos cálculos realizados.

# Pacote

"ros2 run gazebo_sim gazebo_sim"

# Setup

Ambiente ubuntu com ros2 e gazebo. Além disso, é necessário executar o "empty_world", como consta no vídeo.

"ros2 launch turtlebot3_gazebo empty_world.launch.py"

# Vídeo:

https://github.com/IsraelNLC/m3GzSim/assets/99210055/e2120f99-3abf-4ac6-a83b-3608f817c245

