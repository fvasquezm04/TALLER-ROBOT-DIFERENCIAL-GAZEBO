import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CinematicaDiferencial(Node):

    def __init__(self):
        super().__init__("cinematica_diferencial")

        # =====================================================
        # Parámetros geométricos
        # =====================================================

        self.declare_parameter("radio_rueda", 0.08)
        self.declare_parameter("separacion_ruedas", 0.36)

        self.radio_rueda = (
            self.get_parameter("radio_rueda")
            .get_parameter_value()
            .double_value
        )

        self.separacion_ruedas = (
            self.get_parameter("separacion_ruedas")
            .get_parameter_value()
            .double_value
        )

        self.validar_parametros()

        # =====================================================
        # Nombres exactos de las articulaciones
        # =====================================================

        self.nombre_rueda_derecha = "right_wheel_joint"
        self.nombre_rueda_izquierda = "left_wheel_joint"

        # =====================================================
        # Pose integrada a partir de las ruedas
        # =====================================================

        self.x_ruedas = 0.0
        self.y_ruedas = 0.0
        self.theta_ruedas = 0.0

        # Última marca temporal válida de joint_states.
        self.tiempo_anterior = None

        # =====================================================
        # Última pose recibida de /odom
        # =====================================================

        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0

        self.odom_recibido = False

        # Se usa para no imprimir cientos de mensajes por segundo.
        self.ultimo_reporte = self.get_clock().now()

        # =====================================================
        # Suscripciones
        # =====================================================

        self.suscripcion_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback_joint_states,
            20,
        )

        self.suscripcion_odom = self.create_subscription(
            Odometry,
            "/odom",
            self.callback_odom,
            20,
        )

        # =====================================================
        # Publicadores
        # =====================================================

        self.publicador_odom_ruedas = self.create_publisher(
            Odometry,
            "/odom_ruedas",
            20,
        )

        self.publicador_error = self.create_publisher(
            Float64MultiArray,
            "/error_odometria",
            20,
        )

        self.get_logger().info(
            "Nodo de cinemática diferencial iniciado."
        )

        self.get_logger().info(
            "Integrando pose desde /joint_states y "
            "comparando con /odom."
        )

        self.get_logger().info(
            f"Radio: {self.radio_rueda:.3f} m | "
            f"Separación: {self.separacion_ruedas:.3f} m"
        )

    def validar_parametros(self):
        if self.radio_rueda <= 0.0:
            raise ValueError(
                "El radio de la rueda debe ser mayor que cero."
            )

        if self.separacion_ruedas <= 0.0:
            raise ValueError(
                "La separación entre ruedas debe ser mayor que cero."
            )

    # =========================================================
    # Lectura de la odometría de referencia
    # =========================================================

    def callback_odom(self, mensaje):
        self.x_odom = mensaje.pose.pose.position.x
        self.y_odom = mensaje.pose.pose.position.y

        quaternion = mensaje.pose.pose.orientation

        self.theta_odom = self.yaw_desde_quaternion(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )

        self.odom_recibido = True

    # =========================================================
    # Lectura e integración de las velocidades de ruedas
    # =========================================================

    def callback_joint_states(self, mensaje):
        if not mensaje.velocity:
            return

        try:
            indice_derecha = mensaje.name.index(
                self.nombre_rueda_derecha
            )

            indice_izquierda = mensaje.name.index(
                self.nombre_rueda_izquierda
            )

        except ValueError:
            self.get_logger().warning(
                "No se encontraron ambas ruedas en /joint_states."
            )
            return

        if (
            indice_derecha >= len(mensaje.velocity)
            or indice_izquierda >= len(mensaje.velocity)
        ):
            return

        omega_derecha = mensaje.velocity[indice_derecha]
        omega_izquierda = mensaje.velocity[indice_izquierda]

        tiempo_actual = self.tiempo_desde_header(mensaje)

        if self.tiempo_anterior is None:
            self.tiempo_anterior = tiempo_actual
            return

        delta_t = tiempo_actual - self.tiempo_anterior
        self.tiempo_anterior = tiempo_actual

        # Ignorar intervalos inválidos o excesivamente grandes.
        if delta_t <= 0.0 or delta_t > 0.5:
            return

        # -----------------------------------------------------
        # Matriz rueda -> velocidad del cuerpo
        #
        # [v]       [ r/2    r/2  ] [omega_D]
        # [w]   =   [ r/L   -r/L  ] [omega_I]
        # -----------------------------------------------------

        velocidad_lineal = (
            self.radio_rueda / 2.0
        ) * (
            omega_derecha + omega_izquierda
        )

        velocidad_angular = (
            self.radio_rueda / self.separacion_ruedas
        ) * (
            omega_derecha - omega_izquierda
        )

        # -----------------------------------------------------
        # Integración con orientación en el punto medio.
        # -----------------------------------------------------

        theta_media = (
            self.theta_ruedas
            + 0.5 * velocidad_angular * delta_t
        )

        self.x_ruedas += (
            velocidad_lineal
            * math.cos(theta_media)
            * delta_t
        )

        self.y_ruedas += (
            velocidad_lineal
            * math.sin(theta_media)
            * delta_t
        )

        self.theta_ruedas += velocidad_angular * delta_t
        self.theta_ruedas = self.normalizar_angulo(
            self.theta_ruedas
        )

        self.publicar_odom_ruedas(
            mensaje,
            velocidad_lineal,
            velocidad_angular,
        )

        if self.odom_recibido:
            self.publicar_error()

    # =========================================================
    # Publicación de la odometría calculada
    # =========================================================

    def publicar_odom_ruedas(
        self,
        mensaje_joint_states,
        velocidad_lineal,
        velocidad_angular,
    ):
        mensaje_odom = Odometry()

        mensaje_odom.header.stamp = (
            mensaje_joint_states.header.stamp
        )

        mensaje_odom.header.frame_id = "odom"
        mensaje_odom.child_frame_id = "base_footprint_ruedas"

        mensaje_odom.pose.pose.position.x = self.x_ruedas
        mensaje_odom.pose.pose.position.y = self.y_ruedas
        mensaje_odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.quaternion_desde_yaw(
            self.theta_ruedas
        )

        mensaje_odom.pose.pose.orientation.x = qx
        mensaje_odom.pose.pose.orientation.y = qy
        mensaje_odom.pose.pose.orientation.z = qz
        mensaje_odom.pose.pose.orientation.w = qw

        mensaje_odom.twist.twist.linear.x = velocidad_lineal
        mensaje_odom.twist.twist.angular.z = velocidad_angular

        self.publicador_odom_ruedas.publish(mensaje_odom)

    # =========================================================
    # Cálculo y publicación del error
    # =========================================================

    def publicar_error(self):
        error_x = self.x_ruedas - self.x_odom
        error_y = self.y_ruedas - self.y_odom

        error_theta = self.normalizar_angulo(
            self.theta_ruedas - self.theta_odom
        )

        error_posicion = math.sqrt(
            error_x ** 2 + error_y ** 2
        )

        mensaje_error = Float64MultiArray()

        # Orden:
        # [error_x, error_y, error_theta, error_posicion]
        mensaje_error.data = [
            error_x,
            error_y,
            error_theta,
            error_posicion,
        ]

        self.publicador_error.publish(mensaje_error)

        ahora = self.get_clock().now()

        tiempo_desde_reporte = (
            ahora - self.ultimo_reporte
        ).nanoseconds / 1e9

        if tiempo_desde_reporte >= 1.0:
            self.get_logger().info(
                "Pose ruedas: "
                f"x={self.x_ruedas:.3f}, "
                f"y={self.y_ruedas:.3f}, "
                f"theta={self.theta_ruedas:.3f} | "
                "Pose DiffDrive: "
                f"x={self.x_odom:.3f}, "
                f"y={self.y_odom:.3f}, "
                f"theta={self.theta_odom:.3f} | "
                "Error: "
                f"posición={error_posicion:.4f} m, "
                f"orientación={error_theta:.4f} rad"
            )

            self.ultimo_reporte = ahora

    # =========================================================
    # Utilidades temporales
    # =========================================================

    def tiempo_desde_header(self, mensaje):
        segundos = mensaje.header.stamp.sec
        nanosegundos = mensaje.header.stamp.nanosec

        tiempo = segundos + nanosegundos * 1e-9

        # Si el mensaje no trae una marca temporal útil,
        # utilizar el reloj actual del nodo.
        if tiempo <= 0.0:
            tiempo = self.get_clock().now().nanoseconds * 1e-9

        return tiempo

    # =========================================================
    # Utilidades de orientación
    # =========================================================

    @staticmethod
    def normalizar_angulo(angulo):
        return math.atan2(
            math.sin(angulo),
            math.cos(angulo),
        )

    @staticmethod
    def quaternion_desde_yaw(yaw):
        mitad = yaw / 2.0

        return (
            0.0,
            0.0,
            math.sin(mitad),
            math.cos(mitad),
        )

    @staticmethod
    def yaw_desde_quaternion(x, y, z, w):
        seno_yaw = 2.0 * (w * z + x * y)

        coseno_yaw = 1.0 - 2.0 * (
            y * y + z * z
        )

        return math.atan2(seno_yaw, coseno_yaw)


def main(args=None):
    rclpy.init(args=args)

    nodo = CinematicaDiferencial()

    try:
        rclpy.spin(nodo)

    except KeyboardInterrupt:
        pass

    finally:
        nodo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()