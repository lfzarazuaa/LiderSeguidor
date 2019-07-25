/**
 * @file /include/gui_lider_seguidor/main_window.hpp
 *
 * @brief Qt based gui for gui_lider_seguidor.
 *
 * @date November 2010
 **/
#ifndef gui_lider_seguidor_MAIN_WINDOW_H
#define gui_lider_seguidor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>//Incluye QtGui
#include "ui_main_window.h"//Incluye las definiciones de main_window.
#include "qnode.hpp"//Incluye las definiciones de qnode.
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <stdlib.h>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gui_lider_seguidor {//darle como nombre gui_lider_seguidor

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {//Clase Main window que extiende QMainWindow
Q_OBJECT

public: //Miembros y métodos públicos.
        MainWindow(int argc, char** argv, QWidget *parent = 0);//Contruye MainWindow.
        ~MainWindow();//Destructor de MainWindow.
        //Declarar prototipos de funciones.
        void ReadSettings(); //Cargar la configuración del programa qt al inicio.
        void WriteSettings(); // Salvar las configuraciones del programa al cerrar.
        void closeEvent(QCloseEvent *event); // Sobrecargar función.
        void showNoMasterMessage();//Funcion mostrar no hay roscore.

public Q_SLOTS://Eventos que ocurren en la interfaz
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
        void on_actionAbout_triggered();//Función al presionar el menu on on_actionAbout.
        void on_button_connect_clicked(bool check );//Función al hacer click en el boton connect.
        void on_checkbox_use_environment_stateChanged(int state);//FUnción al dar click en el checkbox use enviroment.

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void on_button_tr_clicked();
    void pushButton_Graficar_clicked();
    void pushButton_Limpiar_clicked();
    void pushButton_Guardar_Datos_clicked();
    void radioButton_Encendido_clicked();
    void Grupobtn_clicked(int id);
    void pushButton_Idioma_clicked();
    //Función para actualizar la vista de ingreso.
private:
        Ui::MainWindowDesign ui;//Objeto ui (elementos de ui).
        QNode qnode;//Declarar objeto qnode de la clase Qnode.
        QButtonGroup *Grupobtn;
        QPixmap tb3,ty1,ty2,ty3;
        enum TipoIdioma{Espanol,Ingles};
        TipoIdioma Idioma;
        bool graph_Bool,clear_graph_Bool,save_data_Bool;
        //QCustomPlot *mPlot;
};

}  // namespace gui_lider_seguidor

#endif // gui_lider_seguidor_MAIN_WINDOW_H
