import rclpy
import pytest
from std_msgs.msg import String
from time import sleep
import time


class TestRepublisherListening:
    def setup_method(self):
        # Inicializar rclpy y configurar el nodo y las suscripciones
        rclpy.init()

        # Crear un nodo de prueba para suscribirse al 'republished_topic'
        self.node = rclpy.create_node('testing')

        # Variable para almacenar el mensaje recibido desde el 'republished_topic'
        self.received_message = None

        # Crear una suscripción al 'republished_topic'
        self.test_subscription = self.node.create_subscription(
            String,
            'republished_topic',
            self.subscription_callback,
            10
        )

        # Crear un publicador para el 'input_topic' (el topic donde publicaremos el mensaje)
        self.test_publisher = self.node.create_publisher(String, 'input_topic', 10)

    def teardown_method(self):
        # Limpiar el nodo
        self.node.destroy_node()
        rclpy.shutdown()

    def subscription_callback(self, msg):
        # Esta función es llamada cuando se recibe un mensaje en 'republished_topic'
        self.received_message = msg.data

    def test_republish_listen_and_publish(self):
        # Publicar un mensaje en el 'input_topic' para que el republisher lo procese
        test_msg = String()
        test_msg.data = 'Testing...testing...1..2..3'
        self.test_publisher.publish(test_msg)

        # Espera activa o giro para asegurarse de que el mensaje haya sido procesado
        timeout = 3  # Seconds to wait for message
        start_time = time.time()

        while self.received_message is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node)

        # Asegurarse de que el mensaje recibido es el esperado
        assert self.received_message == 'Testing...testing...1..2..3', \
            f"Expected 'Testing...testing...1..2..3', but got {self.received_message}"

        # Imprimir éxito
        print("Test passed: Republisher correctly listened and republished the message.")
