/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>//Incluye clase QtGui
#include <QMessageBox>//Incluye clase QMessageBox
#include <iostream>//Incluye iostream
#include "../include/gui_lider_seguidor/main_window.hpp"//Incluye el header main window
#include <stdio.h>
#include <stdlib.h>
//donde se define los eventos que van a tener y prototipos de función.
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_lider_seguidor {//da como nombre gui_lider_seguidor para evitar 2 funciones con el mismo nombre.

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
    , qnode(argc,argv)//Crea el objeto qnode.
{
        ui.setupUi(this); // Conectar todos los eventos de ui asi como definir las configuraciones de la gui.
        graph_Bool=false;
        clear_graph_Bool=false;
        save_data_Bool=false;
        Idioma=Espanol;
        int plotDataSize=100;
        double xData[plotDataSize],yData[plotDataSize];
        for( int index=0; index<plotDataSize; ++index )
            {
                xData[index] = index;
                yData[index] = 2*index*index/10+3*index/10;
            };
        //mPlot = new QCustomPlot(this);
        //ui.verticalLayout_7->addItem(mPlot);
        //setLayout(ui.verticalLayout_7);
        //QPixmap tb3("/home/zarazua_rt/catkin_ws/src/gui_lider_seguidor/resources/images/TB3.png");
        //QPixmap ty1("/home/zarazua_rt/catkin_ws/src/gui_lider_seguidor/resources/images/ty1.png");
        //QPixmap ty2("/home/zarazua_rt/catkin_ws/src/gui_lider_seguidor/resources/images/ty2.png");
        //QPixmap ty3("/home/zarazua_rt/catkin_ws/src/gui_lider_seguidor/resources/images/ty3.png");
        //tb3.load("/gui_lider_seguidor/resources/images/TB3.png");
        tb3.load(":/images/tb3.png");;
        ty1.load(":/images/ty1.png");
        ty2.load(":/images/ty2.png");
        ty3.load(":/images/ty3.png");
        ui.lbl_Imagen->setPixmap(tb3);
        ui.lbl_Imagen->setScaledContents(true);
        Grupobtn = new QButtonGroup(this);
        Grupobtn->addButton(ui.radioButtonT1);
        Grupobtn->addButton(ui.radioButtonT2);
        Grupobtn->addButton(ui.radioButtonT3);
        Grupobtn->setExclusive(true);
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); //Conectar la señal aboutqt triggered a su respectivo método.
    //qapp variable global
    QObject::connect(ui.button_tr, SIGNAL(toggled(bool)), this, SLOT(on_button_tr_clicked()));
    QObject::connect(ui.pushButton_Graficar, SIGNAL(clicked()), this, SLOT(pushButton_Graficar_clicked()));
    QObject::connect(ui.pushButton_Limpiar, SIGNAL(clicked()), this, SLOT(pushButton_Limpiar_clicked()));
    QObject::connect(ui.pushButton_Guardar_Datos, SIGNAL(clicked()), this, SLOT(pushButton_Guardar_Datos_clicked()));
    QObject::connect(ui.pushButton_Idioma, SIGNAL(clicked()), this, SLOT(pushButton_Idioma_clicked()));
    QObject::connect(ui.radioButton_Encendido, SIGNAL(clicked()), this, SLOT(radioButton_Encendido_clicked()));
    QObject::connect(Grupobtn,SIGNAL(buttonClicked(int)),this,SLOT(Grupobtn_clicked(int)));
    //Leer conf.
    ReadSettings(); //Lee la configuracion de la ejecución pasada.
    setWindowIcon(QIcon(":/images/icon.png"));//Coloca el icono de la aplicación.
    ui.tab_manager->setCurrentIndex(0); // Poner el indice del tab manager en 0.
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    //conectar la señal rosShutdown al método para cerrar la ventana.
	/*********************
	** Logging
	**********************/
    ui.view_logging->setModel(qnode.loggingModel());//Configura la lista.
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    //conectar la señal loggingUpdated al método updateLoggingView.

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }//si el checbox de recordar configuraciones esta marcado entonces ejecuta la función.
}

MainWindow::~MainWindow() {}//Destructor de la clase.

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {//Función a ejecutar cuando no hay nodo maestro.
    QMessageBox msgBox;//Crear objeto msgBox para mostrar un mensaje.
    msgBox.setText("No se logro encontrar el nodo ros master.");//Poner el texto.
    msgBox.exec();//Espera a elegir el boton.
    close();//Cierra la aplicación.
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui.checkbox_use_environment->isChecked() ) {//Si la casilla usar ambiente esta marcada
        if ( !qnode.init() ) {//Si la inicialización del nodo da falso
            showNoMasterMessage();//Muestra mensaje de error.
        } else {//Si la incialización da verdadero(inicialización válida).
            ui.button_connect->setEnabled(false);//Desactiva el botón de conectar.
            ui.button_tr->setEnabled(true);//Activa el boton trayectoria.
            ui.radioButtonT1->setEnabled(true);
            ui.radioButtonT2->setEnabled(true);
            ui.radioButtonT3->setEnabled(true);
            ui.radioButton_Encendido->setEnabled(true);
        }
    } else {//Si la casilla casilla usar ambiente esta desmarcada
        //Si la casilla usar ambiente esta marcada
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),//Si la inicialización del nodo da falso
                   ui.line_edit_host->text().toStdString()) ) {//Inicializa el nodo con la dirección del maestro y del host.
            showNoMasterMessage();//Muestra mensaje de error.
        } else {//Si la incialización da verdadero(inicialización válida).
            ui.button_connect->setEnabled(false);//Deshabilita el botón de conectar.
            ui.button_tr->setEnabled(true);
            ui.radioButtonT1->setEnabled(true);
            ui.radioButtonT2->setEnabled(true);
            ui.radioButtonT3->setEnabled(true);
            ui.radioButton_Encendido->setEnabled(true);
            ui.line_edit_master->setReadOnly(true);//Hace de solo lectura la caja de texto del maestro.
            ui.line_edit_host->setReadOnly(true);//Hace de solo lectura la caja de texto del host.
            ui.line_edit_topic->setReadOnly(true);//Hace de solo lectura la caja de texto del tópico.
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {//Si cambia de estado la casilla de usar ambiente.
    bool enabled;//variable enabled
    if ( state == 0 ) {//Si el estado de la casilla es 0.
        enabled = true;//Pone en verdadero enabled.
	} else {
        enabled = false;//Pone en falso enabled.
	}
    ui.line_edit_master->setEnabled(enabled);//Activa o destactiva la caja de texto del maestro.
    ui.line_edit_host->setEnabled(enabled);//Activa o destactiva la caja de texto del host.
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();//Actualiza la posición del ListView.
}

void MainWindow::on_button_tr_clicked()
{
    int num=-(Grupobtn->id(Grupobtn->checkedButton())+1);
    qnode.Cambiar_Trayectoria(num);
}

void MainWindow::pushButton_Graficar_clicked()
{
    graph_Bool=!graph_Bool;
    qnode.Send_graph_Bool(graph_Bool);
    if (Idioma==Espanol){
        if (graph_Bool){
            ui.pushButton_Graficar->setText("Detener de Graficar");}
        else{
            ui.pushButton_Graficar->setText("Empezar a Graficar");}
    }
    else{
        if (graph_Bool){
            ui.pushButton_Graficar->setText("Stop plotting");}
        else{
            ui.pushButton_Graficar->setText("Begin plotting");}
    }
}

void MainWindow::pushButton_Limpiar_clicked()
{
    clear_graph_Bool=!clear_graph_Bool;
    qnode.Send_clear_graph_Bool(clear_graph_Bool);
}

void MainWindow::pushButton_Guardar_Datos_clicked()
{
    save_data_Bool=!save_data_Bool;
    qnode.Send_save_data_Bool(save_data_Bool);
}

void MainWindow::radioButton_Encendido_clicked()
{
   bool state=ui.radioButton_Encendido->isChecked();
   qnode.Enviar_Encendido(state);
    if (Idioma==Espanol){
        if(state){
            ui.radioButton_Encendido->setText("Apagar");}
        else{
            ui.radioButton_Encendido->setText("Encender");}
    }
    else{
        if (state){
            ui.radioButton_Encendido->setText(QString::fromUtf8("Turn off"));}
        else{
            ui.radioButton_Encendido->setText(QString::fromUtf8("Turn on"));}
    }

}

void MainWindow::Grupobtn_clicked(int id)
{
  int num=-(Grupobtn->id(Grupobtn->checkedButton())+1);
  switch (num)
  {
   case 1:
      ui.lbl_Imagen->setPixmap(ty1);
    break;
   case 2:
      ui.lbl_Imagen->setPixmap(ty2);
    break;
   case 3:
      ui.lbl_Imagen->setPixmap(ty3);
    break;
   default:
      ui.lbl_Imagen->setPixmap(tb3);
    break;
  }
  //ui.lbl_Imagen->setText(QString::number(num));
  ui.lbl_Imagen->setScaledContents(true);
}

void MainWindow::pushButton_Idioma_clicked()
{
    bool state=ui.radioButton_Encendido->isChecked();
    if (Idioma==Espanol){
        ui.pushButton_Idioma->setText(QString::fromUtf8("Español (Spanish)"));
        Idioma=Ingles;
        if (ui.button_connect->isEnabled()==false){
           qnode.Send_change_language_Bool(true);
        }
        ui.pushButton_Graficar->setText("Begin to Graph");
        ui.pushButton_Limpiar->setText("Clean Graph");
        ui.pushButton_Guardar_Datos->setText("Save data");
        ui.checkbox_use_environment->setText(QString::fromUtf8("Use enviroment variables"));
        ui.checkbox_remember_settings->setText(QString::fromUtf8("Remember start settings"));
        ui.button_connect->setText(QString::fromUtf8("Connect"));
        ui.quit_button->setText(QString::fromUtf8("Quit"));
        ui.lbl_Seleccionar_Trayectoria->setText(QString::fromUtf8("Select Trayectory"));
        ui.radioButtonT1->setText(QString::fromUtf8("Trayectory 1"));
        ui.radioButtonT2->setText(QString::fromUtf8("Trayectory 2"));
        ui.radioButtonT3->setText(QString::fromUtf8("Trayectory 3"));
        if (state){
            ui.radioButton_Encendido->setText(QString::fromUtf8("Turn off"));}
        else{
            ui.radioButton_Encendido->setText(QString::fromUtf8("Turn on"));}
        ui.button_tr->setText(QString::fromUtf8("Load Trayectory"));
        ui.groupBox_12->setTitle(QString::fromUtf8("Messages"));
        ui.tab_manager->setTabText(ui.tab_manager->indexOf(ui.tab_trayectoria), QApplication::translate("MainWindowDesign", "Trayectories", 0, QApplication::UnicodeUTF8));
        ui.tab_manager->setTabText(ui.tab_manager->indexOf(ui.tab_status), QApplication::translate("MainWindowDesign", "ROS Comunications", 0, QApplication::UnicodeUTF8));
        ui.tab_manager->setTabText(ui.tab_manager->indexOf(ui.tab_graficas), QApplication::translate("MainWindowDesign", "Graphs", 0, QApplication::UnicodeUTF8));
        ui.action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        //ui.action_Preferences->setIconText(QApplication::translate("MainWindowDesign", "Preferences", 0, QApplication::UnicodeUTF8));
        ui.actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        ui.actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        ui.action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        ui.dock_status->setWindowTitle(QApplication::translate("MainWindowDesign", "Control Panel", 0, QApplication::UnicodeUTF8));
    }
    else{
        ui.pushButton_Idioma->setText(QString::fromUtf8("Inglés (English)"));
        Idioma=Espanol;
        if (ui.button_connect->isEnabled()==false){
           qnode.Send_change_language_Bool(false);
        }
        ui.pushButton_Graficar->setText(QString::fromUtf8("Empezar a Graficar"));
        ui.pushButton_Limpiar->setText(QString::fromUtf8("Limpiar Gráfica"));
        ui.pushButton_Guardar_Datos->setText(QString::fromUtf8("Guardar datos de la Gráfica"));
        ui.checkbox_use_environment->setText(QString::fromUtf8("Usar variables de entorno"));
        ui.checkbox_remember_settings->setText(QString::fromUtf8("Recordar configuracion de inicio"));
        ui.button_connect->setText(QString::fromUtf8("Conectar"));
        ui.quit_button->setText(QString::fromUtf8("Salir"));
        ui.lbl_Seleccionar_Trayectoria->setText(QString::fromUtf8("Seleccionar Trayectoria"));
        ui.radioButtonT1->setText(QString::fromUtf8("Trayectoria 1"));
        ui.radioButtonT2->setText(QString::fromUtf8("Trayectoria 2"));
        ui.radioButtonT3->setText(QString::fromUtf8("Trayectoria 3"));
        if (state){
            ui.radioButton_Encendido->setText(QString::fromUtf8("Apagar"));}
        else{
            ui.radioButton_Encendido->setText(QString::fromUtf8("Encender"));}
        ui.button_tr->setText(QString::fromUtf8("Cargar Trayectoria"));
        ui.groupBox_12->setTitle(QString::fromUtf8("Mensajes"));
        ui.tab_manager->setTabText(ui.tab_manager->indexOf(ui.tab_trayectoria), QApplication::translate("MainWindowDesign", "Trayectorias", 0, QApplication::UnicodeUTF8));
        ui.tab_manager->setTabText(ui.tab_manager->indexOf(ui.tab_status), QApplication::translate("MainWindowDesign", "Comunicaciones de ROS", 0, QApplication::UnicodeUTF8));
        ui.tab_manager->setTabText(ui.tab_manager->indexOf(ui.tab_graficas), QApplication::translate("MainWindowDesign", "Gr\303\241ficas", 0, QApplication::UnicodeUTF8));
        ui.action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferencias", 0, QApplication::UnicodeUTF8));
        //ui.action_Preferences->setIconText(QApplication::translate("MainWindowDesign", "Preferencias", 0, QApplication::UnicodeUTF8));
        ui.actionAbout->setText(QApplication::translate("MainWindowDesign", "&Acerca de", 0, QApplication::UnicodeUTF8));
        ui.actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "Acerca de &Qt", 0, QApplication::UnicodeUTF8));
        ui.action_Quit->setText(QApplication::translate("MainWindowDesign", "&Salir", 0, QApplication::UnicodeUTF8));
        ui.dock_status->setWindowTitle(QApplication::translate("MainWindowDesign", "Panel de Control", 0, QApplication::UnicodeUTF8));
    }
}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("Acerca de ..."),tr("<h2>Lider-Seguidor(Lider-Follower)</h2><p>Copyright UPIITA</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {//Leer Configuraciones
    QSettings settings("Qt-Ros Package", "gui_lider_seguidor");//Lee la configuración de la GUI en su ejecución pasada.
    restoreGeometry(settings.value("geometry").toByteArray());//Restaura geometria.
    restoreState(settings.value("windowState").toByteArray());//Restaura el estado del widget.
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString(); //Da la dirección del maestro la segunda es la default.
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();//Da la dirección del host la segunda es la default.
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);//Carga la dirección del maestro a la caja de texto del maestro.
    ui.line_edit_host->setText(host_url);//Carga la dirección del maestro a la caja de texto del host.
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();//Obtiene el valor del bit remember
    ui.checkbox_remember_settings->setChecked(remember);//Pasa el valor de remember a recordar configuraciones.
    bool checked = settings.value("use_environment_variables", false).toBool();//Obtiene el valor del bit checked
    ui.checkbox_use_environment->setChecked(checked);//Pasa el valor a la casilla usar ambiente
    if ( checked ) {//Si fue usa el ambiente
        ui.line_edit_master->setEnabled(false);//Deshabilita la caja de texto del maestro.
        ui.line_edit_host->setEnabled(false);//Deshabilita la caja de texto del Host.
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {//Escribir configuraciones.
    QSettings settings("Qt-Ros Package", "gui_lider_seguidor");//Crea el objeto settings.
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    qnode.Enviar_Encendido(false);
    for (int c = 1; c <= 32767; c++){
          for (int d = 1; d <= 2500; d++)
          {}
    }
}

void MainWindow::closeEvent(QCloseEvent *event)//Función cuando se cierra la ventana.
{
    WriteSettings();//Escribe las configuraciones.
    QMainWindow::closeEvent(event);//Cierra la ventana.
}

}  // namespace gui_lider_seguidor

