/**
 * @file /include/gui_lider_seguidor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
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
#include <ros/ros.h>//Incluye la libreria de ROS
#endif
#include <string>//Incluye la clase string.
#include <QThread>//Clase de hilos de ejecucion
#include <QStringListModel>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_lider_seguidor {//Lo define dentro del namespace gui_lider_seguidor para evitar repeticiones.

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {//Clase QNode que extiende QThread
    Q_OBJECT
public:
        QNode(int argc, char** argv );//Constructor de clase.
        virtual ~QNode();//Destructor de clase.
        bool init();//inicializar nodo.
        bool init(const std::string &master_url, const std::string &host_url);//inicializar nodo.
        void run();//Sobrecarga función de hilo de ejecución.

	/*********************
	** Logging
	**********************/
        enum LogLevel {//Definir un numero para cada nombre
                 Debug,//0
                 Info,//1
                 Warn,//2
                 Error,//3
                 Fatal//4
	 };
        //crea el objeto loggingModel de tipo QStringListModel.
        QStringListModel* loggingModel() { return &logging_model; }
        //Prototipo de método log.
        void log( const LogLevel &level, const std::string &msg);//Mostrar en Interfaz los resultados.

        void Cambiar_Trayectoria(int num);
        void Enviar_Encendido(bool state);
        void Send_graph_Bool(bool state);
        void Send_clear_graph_Bool(bool state);
        void Send_save_data_Bool(bool state);
        void Send_change_language_Bool(bool state);

Q_SIGNALS:
        //Señales loggingUpdated y rosShutdown.
        void loggingUpdated();
        void rosShutdown();

private:
        int init_argc;//Valores de parámetros.
	char** init_argv;
        ros::Publisher chatter_publisher,save_data_publisher,change_language_publisher;//Declarar Objeto chatter_publisher tipo Publisher.
        ros::Publisher tr_publisher,state_publisher,graph_publisher,clear_graph_publisher;//Declarar Objeto tr_publisher tipo Publisher.
        std_msgs::Int32 msg_num;
        std_msgs::Bool graph_Bool,clear_graph_Bool,save_data_Bool,change_language_Bool;
        QStringListModel logging_model;//Crear Objeto logging_model tipo QStringListModel.
        //para copiarlo en la lista de la interfaz gráfica.
};

}  // namespace gui_lider_seguidor

#endif /* gui_lider_seguidor_QNODE_HPP_ */
