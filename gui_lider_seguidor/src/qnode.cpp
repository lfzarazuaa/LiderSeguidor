/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include "../include/gui_lider_seguidor/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_lider_seguidor {//Define el espacio de trabajo como gui_lider_seguidor.

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) ://Define el constructor.
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {//Define el destructor.
    if(ros::isStarted()) {//Si ros se ejecuto.
      ros::shutdown(); // Apaga el proceso de ros.
      ros::waitForShutdown();//Espera para el apagado.
    }
    wait();//Espera.
}

bool QNode::init() {//Constructor sin direcciones ip.
    ros::init(init_argc,init_argv,"gui_lider_seguidor");//Inicializa el nodo.
    if ( ! ros::master::check() ) {//Verifica si roscore esta ejecutado.
        return false;//Si no se ejecuta termina la función con falso.
	}
    ros::start(); //Comienza ros de este nodo.
    ros::NodeHandle n;//Declara el nodo n.
    // Agregar aqui los tópicos o servicios.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    tr_publisher = n.advertise<std_msgs::Int32>("lane",1000);
    state_publisher = n.advertise<std_msgs::Bool>("turn_on",1000);
    graph_publisher = n.advertise<std_msgs::Bool>("graph",1000);
    clear_graph_publisher = n.advertise<std_msgs::Bool>("clear_graph",1000);
    save_data_publisher = n.advertise<std_msgs::Bool>("save_data",1000);
    change_language_publisher = n.advertise<std_msgs::Bool>("change_language",1000);
    start();//Ejecuta hilo de ejecución (no se queda ciclado).
    return true;//regresa verdadero a la función.
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {//Constructor con direcciones ip.
    std::map<std::string,std::string> remappings;//Crea objeto tipo mapa (usa como indice la llave).
    remappings["__master"] = master_url;//Define la llave master (indice identificador),  con la dirección del maestro.
    remappings["__hostname"] = host_url;//Define la llave host (indice identificador),  con la dirección del host.
    ros::init(remappings,"gui_lider_seguidor");//Inicializa el nodo con la ip del maestro y del host.
    if ( ! ros::master::check() ) {//Verifica si el maestro esta corriendo.
        return false;//Termina la función y regresa si no lo hace.
	}
    ros::start(); // Evita el cierre del nodo.
    ros::NodeHandle n;//Crea nodo n.
    // Agregar aqui los tópicos o servicios.
    chatter_publisher = n.advertise<std_msgs::String>("mensajes_gui", 1000);
    tr_publisher = n.advertise<std_msgs::Int32>("lane",1000);
    state_publisher = n.advertise<std_msgs::Bool>("turn_on",1000);
    graph_publisher = n.advertise<std_msgs::Bool>("graph",1000);
    clear_graph_publisher = n.advertise<std_msgs::Bool>("clear_graph",1000);
    save_data_publisher = n.advertise<std_msgs::Bool>("save_data",1000);
    change_language_publisher = n.advertise<std_msgs::Bool>("change_language",1000);
    start();//Ejecuta hilo de ejecución (no se queda ciclado).
    return true;//Regresa verdadero para salir de la función.
}

void QNode::run() {//Se llama cuando se ejecuta start en node::init
    ros::Rate loop_rate(2);//Frecuencia de 2Hz de espera.
    int count = 0,cont=10;//Inicia el conteo en 0.
    if (ros::ok()){
        std_msgs::Bool encendido;
        encendido.data=false;
        state_publisher.publish(encendido);
        graph_Bool.data=false;
        graph_publisher.publish(graph_Bool);
        clear_graph_Bool.data=false;
        clear_graph_publisher.publish(clear_graph_Bool);
        save_data_Bool.data=false;
        save_data_publisher.publish(save_data_Bool);
        change_language_Bool.data=false;
        change_language_publisher.publish(change_language_Bool);
        Cambiar_Trayectoria(0);
    }
    while ( ros::ok() && count<cont) {//Mientras la ejecución sea correcta
        //y el conteo sean menor a 101 se manda el mensaje.
        std_msgs::String msg;//Crea el mensaje para ros.
        std::stringstream ss;//Crea el mensaje para almacenarlo en una variable.
        ss << "Conteo (count)= " << count;//Guarda hola mundo en ss.
        msg.data = ss.str();//Guarda el mensaje en la variable a publicar.
        chatter_publisher.publish(msg);//Publica el mensaje.
        log(Debug,std::string(" ")+msg.data);//Muestra el mensaje en la interfaz.
        ros::spinOnce();//Espera a que la información sea enviada.
        loop_rate.sleep();//Hace un retardo de 1 segundo.
        //
        ++count;//Incrementa el conteo.
	}
    std_msgs::String msg;//Crea el mensaje para ros.
    std::stringstream ss;//Crea el mensaje para almacenarlo en una variable.
    if (count==cont){
       ss << "Prueba exitosa (Succesful test).";//Guarda el mensaje en ss.
       msg.data = ss.str();//Guarda el mensaje en la varable a publicar.
       chatter_publisher.publish(msg);//Publica el mensaje.
       log(Debug,std::string(" ")+msg.data);//Muestra el mensaje en la interfaz.
      }
    else{
       ss << "Error al enviar mensaje (Error sending message).";
       msg.data = ss.str();//Guarda el mensaje en la varable a publicar.
       chatter_publisher.publish(msg);//Publica el mensaje.
       log(Error,std::string(" ")+msg.data);//Muestra el mensaje en la interfaz.
    }
    //Q_EMIT rosShutdown(); // Emite la señal para cerrar la gui.
}


void QNode::log( const LogLevel &level, const std::string &msg) {//Función para mostrar mensaje en la interfaz.
    logging_model.insertRows(logging_model.rowCount(),1);//
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::Cambiar_Trayectoria(int num)
{
    std_msgs::String msg;//Crea el mensaje para ros.
    std::stringstream ss;//Crea el mensaje para almacenarlo en una variable.
    msg_num.data=num;//Guardar el dato publicar.
    tr_publisher.publish(msg_num);//Publica el mensaje.
    ss << "Trayectoria (Trayectory): " <<num;//Guarda el mensaje en ss.
    msg.data = ss.str();//Guarda el mensaje en la varable a publicar.
    chatter_publisher.publish(msg);//Publica el mensaje.
    log(Info,msg.data);//Muestra el mensaje en la interfaz.
}

void QNode::Enviar_Encendido(bool state)
{
    std_msgs::Bool encendido;
    encendido.data=state;
    state_publisher.publish(encendido);
}

void QNode::Send_graph_Bool(bool state)
{
    graph_Bool.data=state;
    graph_publisher.publish(graph_Bool);
}

void QNode::Send_clear_graph_Bool(bool state)
{
    clear_graph_Bool.data=state;
    clear_graph_publisher.publish(clear_graph_Bool);
}

void QNode::Send_save_data_Bool(bool state)
{
    save_data_Bool.data=state;
    save_data_publisher.publish(save_data_Bool);
}

void QNode::Send_change_language_Bool(bool state)
{
    change_language_Bool.data=state;
    change_language_publisher.publish(change_language_Bool);
}

}  // namespace gui_lider_seguidor
