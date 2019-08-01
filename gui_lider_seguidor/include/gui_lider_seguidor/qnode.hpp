/**
 * @file /include/gui_lider_seguidor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date Mayo 2019
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gui_lider_seguidor_QNODE_HPP_
#define gui_lider_seguidor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h> // Incluye la libreria de ROS con el namespace ros.
#endif
#include <string> // Incluye la clase string.
#include <QThread> // Clase de hilos de ejecucion en Qt.
#include <QStringListModel> // Incluye la clase QStringListModel.
#include <std_msgs/Int32.h> // Incluye el mensaje de ROS Int32.
#include <std_msgs/String.h> // Incluye el mensaje de ROS String.
#include <std_msgs/Bool.h> // Incluye el mensaje de ROS Bool.
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_lider_seguidor { // Lo define dentro del namespace gui_lider_seguidor para evitar repeticiones.

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread { // Clase QNode que extiende QThread.
    Q_OBJECT
public:
        QNode(int argc, char** argv ); // Constructor de clase.
        virtual ~QNode(); // Destructor de clase.
        bool init(); // Método para inicializar el nodo.
        bool init(const std::string &master_url, const std::string &host_url); // Método para inicializar nodo con url.
        void init_node();
        void run(); // Sobrecarga el método de hilo de ejecución.

	/*********************
	** Logging
	**********************/

        enum LogLevel { // Definir un número para cada nombre con el enum LogLevel.
                 Debug, // 0
                 Info, // 1
                 Warn, // 2
                 Error, // 3
                 Fatal // 4
	 };
        // Crea el objeto loggingModel de tipo QStringListModel.
        QStringListModel* loggingModel() { return &logging_model; }
        // Prototipo del método log.
        void log( const LogLevel &level, const std::string &msg); // Mostrar en la Interfaz gráfica los resultados en una lista.
        void show_info(const std::string ss); // Mostrar un texto en la interfaz gráfica asi como en los mensajes de ROS info.
        void Cambiar_Trayectoria(int num); // Definición del método para cambiar la trayectoria por medio de un tópico de ROS.
        void Enviar_Encendido(bool state); // Definición del método para enviar apagado/encendido por medio de un tópico de ROS.
        void Send_graph_Bool(bool state); // Definición del método para graficar por medio de un tópico de ROS.
        void Send_clear_graph_Bool(bool state); // Definición del método para limpiar la gráfica por medio de un tópico de ROS.
        void Send_save_data_Bool(bool state); // Definición del método para salvar los puntos alcanzados por medio de un tópico de ROS.
        void Send_change_language_Bool(bool state); // Definición del método para cambiar el idioma por medio de un tópico de ROS.

Q_SIGNALS:
        //Señales loggingUpdated y rosShutdown.
        void loggingUpdated(); // Definición del método para actualizar la lista.
        void rosShutdown(); // Definición para cerrar ROS.

private:
        int init_argc; // Valores de parámetros de entrada.
        char** init_argv; // Inicializa argumento.
        ros::Publisher chatter_publisher,save_data_publisher,change_language_publisher; // Declara objetos para publicar mensajes.
        ros::Publisher tr_publisher,state_publisher,graph_publisher,clear_graph_publisher; // Declara objetos para publicar mensajes.
        std_msgs::Int32 msg_num; // Declara los mensajes a publicar de tipo entero.
        std_msgs::Bool encendido,graph_Bool,clear_graph_Bool,save_data_Bool,change_language_Bool; // Declara los mensajes a publicar de tipo bool.
        QStringListModel logging_model; // Crear objeto logging_model tipo QStringListModel para copiarlo en la lista de la interfaz gráfica.
};

}  // namespace gui_lider_seguidor

#endif /* gui_lider_seguidor_QNODE_HPP_ */
