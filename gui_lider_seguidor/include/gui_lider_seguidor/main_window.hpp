/**
 * @file /include/gui_lider_seguidor/main_window.hpp
 *
 * @brief Interfaz gráfica para lider seguidor.
 *
 * @date Mayo 2019
 **/
#ifndef gui_lider_seguidor_MAIN_WINDOW_H
#define gui_lider_seguidor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow> // Incluye la librería QtGui de elementos de interfaz.
#include "ui_main_window.h" // Incluye las definiciones de main_window donde se tiene el diseño de la interfaz.
#include "qnode.hpp" // Incluye las definiciones de qnode clase que contiene funcionalidad con ROS.
#include <std_msgs/Int32.h> // Incluye mensaje de ROS.
#include <stdio.h> // Funciones de entrada y salida.
#include <stdlib.h> // Funciones estándar para programación.
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gui_lider_seguidor { // Darle como nombre gui_lider_seguidor al espacio de trabajo para diferenciarlo de otros.

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt Lider-Seguidor.
 */
class MainWindow : public QMainWindow { // Clase MainWindow que extiende QMainWindow.
Q_OBJECT

public: // Miembros y métodos públicos.
        MainWindow(int argc, char** argv, QWidget *parent = 0); // Definición del constructor de la clase MainWindow.
        ~MainWindow(); // Definición del destructor de la clase MainWindow.
        // Declarar prototipos de funciones.
        void ReadSettings(); // Cargar la configuración de la última sesión.
        void WriteSettings(); // Salvar las configuraciones del programa al cerrar.
        void closeEvent(QCloseEvent *event); // Sobrecargar función closeEvent para agregarle mas funcionalidad.
        void showNoMasterMessage(); // Función mostrar que no se ejecuto el roscore.

public Q_SLOTS: // Eventos que ocurren en la interfaz.
        /******************************************
        ** Auto-connections (connectSlotsByName())
        *******************************************/
        void on_actionAbout_triggered(); // Método al presionar el menú on_actionAbout.
        void on_button_connect_clicked(bool check ); // Método al hacer click en el botón button_connect.
        void on_checkbox_use_environment_stateChanged(int state); // Método al dar click en el checkbox use enviroment.

        /******************************************
        ** Manual connections
        *******************************************/
        void updateLoggingView(); // Método para actualizar la lista de posición automáticamente.
        void on_button_tr_clicked(); // Método para a ejecutar cuando de da click en el botón button_tr el cual cambia la tayectoria.
        void pushButton_Graficar_clicked(); // Método para empezar a graficar los puntos alcanzados.
        void pushButton_Limpiar_clicked(); // Método para limpiar la gráfica de los puntos graficados.
        void pushButton_Guardar_Datos_clicked(); // Método para Guardar los puntos alcanzados en archivos.
        void radioButton_Encendido_clicked(); // Método para indicar a los robots que avancen.
        void Grupobtn_clicked(int id); // Método para seleccionar trayectoria por medio de las opciones que despliega la interfaz gráfica.
        void pushButton_Idioma_clicked(); // Método para mostrar la información ya sea en español o en inglés.

private:
        Ui::MainWindowDesign ui; // Crea objeto ui (elementos de ui y sus funciones para cambiar etiquetas).
        QNode qnode; // Declarar objeto qnode de la clase Qnode la cual contiene la funcionalidad de ROS para esta interfaz.
        QButtonGroup *Grupobtn; // Crea objeto para el grupo de botones.
        QPixmap tb3,ty1,ty2,ty3; // Carga imágenes del turtlebot y de las trayectorias seleccionadas.
        enum TipoIdioma{Espanol,Ingles}; // Crea variable tipo enum para selecionar el Idioma.
        TipoIdioma Idioma;// Crea variable Idioma de tipo TipoIdioma(enum) para seleccionar el idioma.
        bool graph_Bool,clear_graph_Bool,save_data_Bool; // Crea variables de tipo booleanas que sirven de auxiliares para gráficar.
};

}  // Fin del namespace gui_lider_seguidor

#endif // gui_lider_seguidor_MAIN_WINDOW_H
