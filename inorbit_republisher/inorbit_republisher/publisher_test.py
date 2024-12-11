#This test aims to verify that "republisher.py" file can act like a publisher node
#Message string was choosen based on an awesome beetlejuice song, i am not an alcoholic nor any kind.
import rclpy
import pytest
from std_msgs.msg import String
from time import sleep


class TestRepublisher:

    def subscription_callback(self, msg):
        # Esta función se ejecuta cuando recibimos un mensaje en 'republished_topic'
        self.received_message = msg.data

    def setup_method(self):
        # Inicializa ROS 2
        rclpy.init()
        
        # Crear un nodo de prueba para suscribirse a 'republished_topic'
        self.node = rclpy.create_node('test_republisher')

        # Crear una suscripción al 'republished_topic'
        self.test_subscription = self.node.create_subscription(
            String,
            'republished_topic',  # Este es el topic donde el republisher publica los mensajes
            self.subscription_callback,
            10
        )

    def teardown_method(self):
        # Limpiar el nodo después de la prueba
        self.node.destroy_node()
        rclpy.shutdown()

    def test_republish_message(self):
        # Esperamos el tiempo necesario para que el republisher publique un mensaje
        sleep(1)

        # Procesamos un ciclo de ROS 2 para que el mensaje sea recibido
        rclpy.spin_once(self.node)

        # Ahora que el mensaje debería haber sido republicado, verificamos que el mensaje recibido sea el esperado
        expected_message = 'Work all night on a drink of rum'  # El mensaje que el republisher debería publicar
        assert self.received_message == expected_message, \
            f"Expected '{expected_message}', but got {self.received_message}"

        # Imprimir éxito
        print("Daylight come and me wan' go home")