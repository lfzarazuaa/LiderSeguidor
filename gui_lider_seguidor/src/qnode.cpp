/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date Mayo 2019
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h> // Incluye la libreria de ROS con el namespace ros.
#include <ros/network.h> //Incluye la libreria de ros network.
#include <string> // Incluye la clase string.
#include <std_msgs/String.h> // Incluye el mensaje de ROS String.
#include <std_msgs/Int32.h> // Incluye el mensaje de ROS Int32.
#include <std_msgs/Bool.h> // Incluye el mensaje de ROS Bool.
#include <sstream> // Incluye libreria sstream.
#include "../include/gui_lider_seguidor/qnode.hpp" // Incluye prototipo de la clase qnode.

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_lider_seguidor { // Define el espacio de trabajo como gui_lider_seguidor.

/*****************************************************************************
** Implementación
*****************************************************************************/

QNode::QNode(int argc, char** argv ) : // Define el constructor.
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() { // Define el destructor.
    if(ros::isStarted()) { // Si ros se ejecutó.
      ros::shutdown(); // Apaga el proceso de ros.
      ros::waitForShutdown(); // Espera para el apagado.
    }
    wait(); // Espera.
}

bool QNode::init() { // Inicialización sin direcciones ip.
    ros::init(init_argc,init_argv,"gui_lider_seguidor"); // Inicializa el nodo en ROS con el nombre gui_lider_seguidor.
    if ( ! ros::master::check() ) { // Verifica si roscore o el nodo maestro se está ejecutando.
        return false; // Si no se ejecuta termina la función con falso.
	}
    init_node(); // Llama al método para publicar mensajes con ROS.
    return true; // Regresa verdadero a la función.
}

bool QNode::init(const std::string &master_url, const std::string &host_url) { // Inicialización con direcciones ip.
    std::map<std::string,std::string> remappings; // Crea objeto tipo mapa (usa como índice la llave).
    remappings["__master"] = master_url; // Define la llave master (indice identificador),  con la dirección del maestro.
    remappings["__hostname"] = host_url; // Define la llave host (indice identificador),  con la dirección del host.
    ros::init(remappings,"gui_lider_seguidor"); // Inicializa el nodo con la ip del maestro y del host.
    if ( ! ros::master::check() ) { // Verifica si roscore o el nodo maestro se está ejecutando.
        return false; // Si no se ejecuta termina la función con falso.
	}
    init_node(); // Llama al método para publicar mensajes con ROS.
    return true; // Regresa verdadero para salir de la función.
}

void QNode::init_node() // Método para inciar el nodo sus mensajes.
{
    ros::start(); // Verifica que el nodo este funcionando correctamente.
    ros::NodeHandle n; // Crea el manejador del nodo.
    // Agregar aqui los tópicos o servicios.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000); // Crea publicador de mensaje para el tópico chatter.
    tr_publisher = n.advertise<std_msgs::Int32>("lane",1000); // Crea publicador de mensaje para el tópico lane.
    state_publisher = n.advertise<std_msgs::Bool>("turn_on",1000); // Crea publicador de mensaje para el tópico turn_on.
    graph_publisher = n.advertise<std_msgs::Bool>("graph",1000); // Crea publicador de mensaje para el tópico chatter.
    clear_graph_publisher = n.advertise<std_msgs::Bool>("clear_graph",1000); // Crea publicador de mensaje para el tópico chatter.
    save_data_publisher = n.advertise<std_msgs::Bool>("save_data",1000); // Crea publicador de mensaje para el tópico chatter.
    change_language_publisher = n.advertise<std_msgs::Bool>("change_language",1000); // Crea publicador de mensaje para el tópico chatter.
    start(); // Ejecuta hilo de ejecución (no se queda ciclado).
}

void QNode::run() { // Se llama cuando se ejecuta start en node::init
    ros::Rate loop_rate(2); // Frecuencia de 2Hz de espera.
    int count = 0,cont=10; // Inicia el conteo en 0.
    if (ros::ok()){ // Si roscore se está ejecutando.
        encendido.data=false; // Coloca el encendido en falso para evitar el avance del robot.
        state_publisher.publish(encendido); // Publica el encendido.
        graph_Bool.data=false; // Coloca la opción para graficar en falso para iniciar sin graficar.
        graph_publisher.publish(graph_Bool); // Publica la opción para graficar.
        clear_graph_Bool.data=false; // Coloca la opción para limpiar gráfica en falso para iniciar con la gráfica limpia.
        clear_graph_publisher.publish(clear_graph_Bool); // Publica la opción para limpiar gráfica.
        save_data_Bool.data=false; // Coloca la opción para salvar gráfica en falso.
        save_data_publisher.publish(save_data_Bool); // Publica la opción para salvar gráfica.
        change_language_Bool.data=false; // Coloca la opción para cambiar idioma en falso.
        change_language_publisher.publish(change_language_Bool); // Publica la opción para cambiar idioma.
        Cambiar_Trayectoria(0); // Cambia a la trayectoria 0.
    }
    while ( ros::ok() && count<cont) { // Mientras la ejecución sea correcta y el conteo sean menor a 10 se manda el mensaje.
        std_msgs::String msg; // Crea el mensaje para ros.
        std::stringstream ss; // Crea el mensaje para almacenarlo en una variable.
        ss << "Conteo (count)= " << count; // Guarda Conteo (count)= en ss.
        msg.data = ss.str(); // Guarda el mensaje en la variable a publicar.
        chatter_publisher.publish(msg); // Publica en el topico chatter lo almacenado en msg.
        log(Debug,std::string(" ")+msg.data); // Muestra el mensaje en la interfaz gráfica.
        ros::spinOnce(); // Espera a que la información sea enviada.
        loop_rate.sleep(); // Hace un retardo de 0.5 segundos.
        ++count; // Incrementa el conteo.
	}
    std_msgs::String msg; // Crea el mensaje para ros.
    std::stringstream ss; // Crea el mensaje para almacenarlo en una variable.
    if (count==cont){ // Si se terminó el conteo
       ss << "Prueba exitosa (Succesful test)."; // Guarda el mensaje en ss.
       msg.data = ss.str(); // Guarda el mensaje en la varable a publicar.
       chatter_publisher.publish(msg); // Publica el mensaje.
       log(Debug,std::string(" ")+msg.data); // Muestra el mensaje en la interfaz.
      }
    else{ // Si no se terminó el conteo
       ss << "Error al enviar mensaje (Error sending message).";
       msg.data = ss.str(); // Guarda el mensaje en la varable a publicar.
       chatter_publisher.publish(msg); // Publica el mensaje.
       log(Error,std::string(" ")+msg.data); // Muestra el mensaje en la interfaz.
    }
    //Q_EMIT rosShutdown(); // Emite la señal para cerrar la gui.
}


void QNode::log( const LogLevel &level, const std::string &msg) { // Método para mostrar mensaje en la interfaz gráfica.
    logging_model.insertRows(logging_model.rowCount(),1); // Inserta una nueva fila para separar el texto.
    std::stringstream logging_model_msg; // Declara la cadena de texto.
    switch ( level ) { // Elige entre las opciones de los enum.
		case(Debug) : {
                ROS_DEBUG_STREAM(msg); // Manda el mensaje por ROS como debug.
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg; // Muestra el mensaje en la interfaz gráfica.
				break;
		}
		case(Info) : {
                ROS_INFO_STREAM(msg); // Manda el mensaje por ROS como info.
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg; // Muestra el mensaje en la interfaz gráfica.
				break;
		}
		case(Warn) : {
                ROS_WARN_STREAM(msg); // Manda el mensaje por ROS como warning.
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg; // Muestra el mensaje en la interfaz gráfica.
				break;
		}
		case(Error) : {
                ROS_ERROR_STREAM(msg); // Manda el mensaje por ROS como error.
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg; // Muestra el mensaje en la interfaz gráfica.
				break;
		}
		case(Fatal) : {
                ROS_FATAL_STREAM(msg); // Manda el mensaje por ROS como fatal.
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg; // Muestra el mensaje en la interfaz gráfica.
				break;
		}
	}
    QVariant new_row(QString(logging_model_msg.str().c_str())); // Guarda la cadena en una nueva fila.
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row); // Agrega la nueva fila al texto.
    Q_EMIT loggingUpdated(); // Reajusta la barra de scroll.
}

void QNode::show_info(const std::string ss) // Método para mostrar una cadena de texto en mensaje info de ROS.
{
    std_msgs::String msg; // Crea el mensaje para ros.
    msg.data = ss; // Guarda el mensaje en la varable a publicar.
    chatter_publisher.publish(msg); // Publica el mensaje.
    log(Info,msg.data); // Muestra el mensaje en la interfaz gráfica.
}

void QNode::Cambiar_Trayectoria(int num) // Método para Cambiar la trayectoria.
{
    msg_num.data=num; // Guardar el dato publicar.
    tr_publisher.publish(msg_num); // Publica el mensaje.
    std::stringstream ss; // Crear variable para concatenar.
    ss << "Trayectoria (Trayectory): " << num; // Concatenar las cadenas de texto.
    show_info(ss.str()); // Mostrar la información en ROS y en la interfaz gráfica.
}

void QNode::Enviar_Encendido(bool state) // Método para encender o apagar el movimiento del robot.
{
    encendido.data=state; // Guarda el dato a publicar.
    state_publisher.publish(encendido); // Publica el dato.
    if (state){show_info("Encendido (Turn on)");} // Mensaje en estado verdadero.
    else{show_info("Apagado (Turn off)");} //Mensaje en estado falso.
}

void QNode::Send_graph_Bool(bool state) // Método para graficar o detener de graficar la información.
{
    graph_Bool.data=state; // Guarda el dato a publicar.
    graph_publisher.publish(graph_Bool); // Publica el dato.
    if (state){show_info("Graficando (Graphing)");} // Mensaje en estado verdadero.
    else{show_info("Sin Graficar (Not Graphing)");} //Mensaje en estado falso.
}

void QNode::Send_clear_graph_Bool(bool state) // Método para limpiar la gráfica.
{
    clear_graph_Bool.data=state; // Guarda el dato a publicar.
    clear_graph_publisher.publish(clear_graph_Bool); // Publica el dato.
    show_info(QString::fromUtf8("Gráfica limpiada (Graph cleared)").toStdString()); // Mensaje.
}

void QNode::Send_save_data_Bool(bool state) // Método para guardar los puntos alcanzados.
{
    save_data_Bool.data=state; // Guarda el dato a publicar.
    save_data_publisher.publish(save_data_Bool); // Publica el dato.
    show_info("Datos salvados (Data saved)"); //Mensaje.
}

void QNode::Send_change_language_Bool(bool state) // Método para cambiar de idioma
{
    change_language_Bool.data=state; // Guarda el dato a publicar.
    change_language_publisher.publish(change_language_Bool); // Publica el dato.
    if (state){show_info(QString::fromUtf8("Idioma en inglés (Language on english)").toStdString());} // Mensaje en estado verdadero.
    else{show_info(QString::fromUtf8("Idioma en español (Language on spanish)").toStdString());} // Mensaje en estado falso.
}

}  // namespace gui_lider_seguidor
