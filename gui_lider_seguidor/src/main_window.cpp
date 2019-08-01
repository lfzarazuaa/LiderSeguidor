/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date Mayo 2019
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui> // Incluye la librería QtGui de elementos de interfaz.
#include <QMessageBox> // Incluye clase QMessageBox para desplegar mensajes por la interfaz.
#include <iostream> // Incluye la clase iostream de utilidades.
#include "../include/gui_lider_seguidor/main_window.hpp" // Incluye el header main window como definicición de prototipo de clase.
#include <stdio.h> // Funciones de entrada y salida.
#include <stdlib.h> // Funciones estándar para programación.
// Donde se definen los eventos que van a tener y prototipos de función.
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_lider_seguidor {// Da como nombre de espacio de trabajo gui_lider_seguidor para identificarlo entre otras ventanas con las mismas funciones.

using namespace Qt; // Define como default el espacio Qt para evitar colocarlo en todo el código.

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
    , qnode(argc,argv) // Constructor del objeto qnode.
{
        ui.setupUi(this); // Conectar todos los eventos de la interfaz asi como definir las configuraciones de la gui.
        graph_Bool=false; // Define en falso la bandera de graficar.
        clear_graph_Bool=false; // Define en falso la bandera de limpiar gráfica (sólo se usa como disparador de evento).
        save_data_Bool=false; // Define en falso la bandera de guardar los puntos alcanzados (sólo se usa como disparador de evento).
        Idioma=Espanol; // Define el idioma con el que se comienza la interfaz gráfica.
        tb3.load(":/images/tb3.png"); // Carga la imagen de inicio a tb3.
        ty1.load(":/images/ty1.png"); // Carga la imagen de la trayectoria 1 a ty1.
        ty2.load(":/images/ty2.png"); // Carga la imagen de la trayectoria 2 a ty2.
        ty3.load(":/images/ty3.png"); // Carga la imagen de la trayectoria 3 a ty3.
        ui.lbl_Imagen->setPixmap(tb3); // Muestra la imagen tb3 en la interfaz gráfica en donde se ubica lbl_Imagen.
        ui.lbl_Imagen->setScaledContents(true); // Hace que lbl_Imagen pueda escalar la imagen segun su tamaño.
        Grupobtn = new QButtonGroup(this); // Crea un Grupo de botones para agupar a los botones y asi poderlos seleccionar sólo uno.
        Grupobtn->addButton(ui.radioButtonT1); // Agrega el botón para seleccionar la trayectoria 1.
        Grupobtn->addButton(ui.radioButtonT2); // Agrega el botón para seleccionar la trayectoria 2.
        Grupobtn->addButton(ui.radioButtonT3); // Agrega el botón para seleccionar la trayectoria 3.
        Grupobtn->setExclusive(true); // Hace exclusivo el grupo de botones para poder seleccionar sólo uno.
        // Conectar la señal aboutqt triggered a su respectivo método, qapp es una variable global.
        QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
        // Conectar la señal toogled del botón button_tr al método on_button_tr_clicked para que cada vez que ocurra este se ejecute.
        QObject::connect(ui.button_tr, SIGNAL(toggled(bool)), this, SLOT(on_button_tr_clicked()));
        // Asociar las acciones al interactuar con la interfaz gráfica a sus métodos a ejecutarse cuando estas acciones ocurran.
        QObject::connect(ui.pushButton_Graficar, SIGNAL(clicked()), this, SLOT(pushButton_Graficar_clicked())); // Da un método a pushButton_Graficar.
        QObject::connect(ui.pushButton_Limpiar, SIGNAL(clicked()), this, SLOT(pushButton_Limpiar_clicked())); // Da un método a pushButton_Limpiar.
        QObject::connect(ui.pushButton_Guardar_Datos, SIGNAL(clicked()), this, SLOT(pushButton_Guardar_Datos_clicked())); // Da un método a pushButton_Guardar_Datos.
        QObject::connect(ui.pushButton_Idioma, SIGNAL(clicked()), this, SLOT(pushButton_Idioma_clicked())); // Da un método a pushButton_Idioma.
        QObject::connect(ui.radioButton_Encendido, SIGNAL(clicked()), this, SLOT(radioButton_Encendido_clicked())); // Da un método a radioButton_Encendido
        QObject::connect(Grupobtn,SIGNAL(buttonClicked(int)),this,SLOT(Grupobtn_clicked(int))); // EJecuta el método al momento de cambiar la selección del grupo de botones.
        //Leer configuraciones.
        ReadSettings(); // Lee la configuración de la ejecución pasada.
        setWindowIcon(QIcon(":/images/icon.png")); // Coloca el ícono de la aplicación.
        ui.tab_manager->setCurrentIndex(0); // Poner el índice del tab manager en 0.
        QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
        // Conectar la señal rosShutdown al método para cerrar la ventana.
        /*********************
        ** Registro
        **********************/
        ui.view_logging->setModel(qnode.loggingModel()); // Asocia el string del log al objeto gráfico.
        QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
        // Conectar la señal loggingUpdated al método updateLoggingView.

        /*********************
        ** Incio Automático
        **********************/
        if ( ui.checkbox_remember_settings->isChecked() ) {
            on_button_connect_clicked(true);
        }// Si el checbox de recordar configuraciones esta marcado entonces ejecuta el método.
}

MainWindow::~MainWindow() {}// Destructor de la clase.

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() { // Función a ejecutar cuando no hay nodo maestro.
    QMessageBox msgBox; // Crear objeto msgBox para mostrar un mensaje en pantalla.
    msgBox.setText("No se logro encontrar el nodo ros master."); // Configurar el texto del mensaje.
    msgBox.exec(); // Espera al usuario que presione el botón aceptar.
    close(); // Cierra la aplicación.
}

void MainWindow::on_button_connect_clicked(bool check) {
    if ( ui.checkbox_use_environment->isChecked() ) { // Si la casilla usar ambiente esta marcada
        if ( !qnode.init() ) { // Si la inicialización del nodo da falso
            showNoMasterMessage(); // Muestra mensaje de error.
        } else { // Si la incialización da verdadero (inicialización válida).
            ui.button_connect->setEnabled(false); // Desactiva el botón de conectar.
            ui.button_tr->setEnabled(true); // Activa el botón de cargar trayectoria.
            ui.radioButtonT1->setEnabled(true); // Activa el botón de trayectoria 1.
            ui.radioButtonT2->setEnabled(true); // Activa el botón de trayectoria 2.
            ui.radioButtonT3->setEnabled(true); // Activa el botón de trayectoria 3.
            ui.radioButton_Encendido->setEnabled(true); // Activa el botón de encender/apagar robots.
        }
    } else { // Si la casilla casilla usar ambiente esta desmarcada
        // Si la casilla usar ambiente esta marcada.
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),// Si la inicialización del nodo da falso.
                   ui.line_edit_host->text().toStdString()) ) { // Inicializa el nodo con la dirección del maestro y del host.
            showNoMasterMessage(); // Muestra mensaje de error.
        } else { // Si la incialización da verdadero(inicialización válida).
            ui.button_connect->setEnabled(false); // Deshabilita el botón de conectar.
            ui.button_tr->setEnabled(true); // Activa el botón de cargar trayectoria.
            ui.radioButtonT1->setEnabled(true); // Activa el botón de trayectoria 1.
            ui.radioButtonT2->setEnabled(true); // Activa el botón de trayectoria 2.
            ui.radioButtonT3->setEnabled(true); // Activa el botón de trayectoria 3.
            ui.radioButton_Encendido->setEnabled(true); // Activa el botón de encender/apagar robots.
            ui.line_edit_master->setReadOnly(true); // Hace de sólo lectura la caja de texto del maestro.
            ui.line_edit_host->setReadOnly(true); // Hace de sólo lectura la caja de texto del host.
            ui.line_edit_topic->setReadOnly(true); // Hace de sólo lectura la caja de texto del tópico.
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) { // Si cambia de estado la casilla de usar ambiente.
    bool enabled; // Define a la variable enabled.
    if ( state == 0 ) { // Si el estado de la casilla es 0.
        enabled = true; // Pone en verdadero enabled.
	} else {
        enabled = false; // Pone en falso enabled.
	}
    ui.line_edit_master->setEnabled(enabled); // Activa o desactiva la caja de texto del maestro.
    ui.line_edit_host->setEnabled(enabled); // Activa o desactiva la caja de texto del host.
}

/*****************************************************************************
** Implementación [Slots][conectados manualmente]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

void MainWindow::updateLoggingView() { // Método para actualizar el texto de la lista.
    ui.view_logging->scrollToBottom(); // Actualiza la posición del ListView.
}

void MainWindow::on_button_tr_clicked() // Método para cambiar la trayectoria.
{
    int num=-(Grupobtn->id(Grupobtn->checkedButton())+1); // Obtiene el número de trayectoria.
    qnode.Cambiar_Trayectoria(num); // Publica la trayectoria en ROS y lo imprime.
}

void MainWindow::pushButton_Graficar_clicked() // Método para indicar que se quieren graficar los datos.
{
    graph_Bool=!graph_Bool; // Obtiene el valor contrario.
    qnode.Send_graph_Bool(graph_Bool); // Publica el dato usando ROS para detener de graficar o graficar.
    if (Idioma==Espanol){ // Si es español se coloca el mensaje de la siguiente acción en español.
        if (graph_Bool){
            ui.pushButton_Graficar->setText("Detener de Graficar");}
        else{
            ui.pushButton_Graficar->setText("Empezar a Graficar");}
    }
    else{ // Si es ingles se coloca el mensaje de la siguiente acción en ingles.
        if (graph_Bool){
            ui.pushButton_Graficar->setText("Stop plotting");}
        else{
            ui.pushButton_Graficar->setText("Begin plotting");}
    }
}

void MainWindow::pushButton_Limpiar_clicked() // Método para limpiar la gráfica.
{
    clear_graph_Bool=!clear_graph_Bool; // Niega el valor solo para verificar que se hizo la acción.
    qnode.Send_clear_graph_Bool(clear_graph_Bool); // Envia la señal que indica que se cambio el dato.
}

void MainWindow::pushButton_Guardar_Datos_clicked() // Método para Guardar los datos.
{
    save_data_Bool=!save_data_Bool; // Niega el valor solo para verificar que se hizo la acción.
    qnode.Send_save_data_Bool(save_data_Bool); // Envia la señal que indica que se cambio el dato.
}

void MainWindow::radioButton_Encendido_clicked() // Método para encender o apagar el movimiento de los robots.
{
   bool state=ui.radioButton_Encendido->isChecked(); // Obtiene el valor del botón  y lo guarda en el estado.
   qnode.Enviar_Encendido(state); // Publica el valor de la variable.
    if (Idioma==Espanol){ // Si es español se coloca el mensaje de la siguiente acción en español.
        if(state){
            ui.radioButton_Encendido->setText("Apagar");}
        else{
            ui.radioButton_Encendido->setText("Encender");}
    }
    else{ // Si es ingles se coloca el mensaje de la siguiente acción en ingles.
        if (state){
            ui.radioButton_Encendido->setText(QString::fromUtf8("Turn off"));}
        else{
            ui.radioButton_Encendido->setText(QString::fromUtf8("Turn on"));}
    }

}

void MainWindow::Grupobtn_clicked(int id) // Método a ejecutar cuando se selecciona alguna trayectoria.
{
  int num=-(Grupobtn->id(Grupobtn->checkedButton())+1); // Obtiene el número de trayectoria.
  switch (num) // Ejecuta un switch case para encontrar la opción deseada.
  {
   case 1: // Si se selecciono trayectoria 1.
      ui.lbl_Imagen->setPixmap(ty1); // Muestra la imagen de la trayectoria 1.
    break;
   case 2: // Si se selecciono trayectoria 2.
      ui.lbl_Imagen->setPixmap(ty2); // Muestra la imagen de la trayectoria 2.
    break;
   case 3: // Si se selecciono trayectoria 3.
      ui.lbl_Imagen->setPixmap(ty3); // Muestra la imagen de la trayectoria 3.
    break;
   default: // Si es otra opción.
      ui.lbl_Imagen->setPixmap(tb3); // Se selecciona la imagen de inicio.
    break;
  }
  ui.lbl_Imagen->setScaledContents(true); // Escala la imagen si es ajusta la ventana.
}

void MainWindow::pushButton_Idioma_clicked() // Método a ejecutar si se cambió el idioma.
{
    bool state=ui.radioButton_Encendido->isChecked(); // Guarda y obtiene el estado del botón.
    if (Idioma==Espanol){ // Si es español lo cambia a inglés.
        ui.pushButton_Idioma->setText(QString::fromUtf8("Español (Spanish)")); // Cambia la opción a elegir español.
        Idioma=Ingles; // Hace el idioma seleccionado Inglés.
        if (ui.button_connect->isEnabled()==false){ // Si ya se inicializó el nodo de ROS.
           qnode.Send_change_language_Bool(true); // Publica el mensaje de idioma.
        }
        // Cambia el texto de los elementos gráficos a Inglés.
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
    else{ // Si es inglés lo cambia a español.
        ui.pushButton_Idioma->setText(QString::fromUtf8("Inglés (English)")); // Cambia la opción a elegir inglés.
        Idioma=Espanol; // Cambia el idioma elegido a español.
        if (ui.button_connect->isEnabled()==false){ // Si ya se inicializó el nodo de ROS.
           qnode.Send_change_language_Bool(false); // Publica el mensaje de idioma.
        }
        // Cambia el texto de los elementos gráficos a Español.
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
** Implementación [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("Acerca de ..."),tr("<h2>Lider-Seguidor(Lider-Follower)</h2><p>Copyright UPIITA</p>"));
}

/*****************************************************************************
** Implementación [Configuración]
*****************************************************************************/

void MainWindow::ReadSettings() { // Método para leer Configuraciones
    QSettings settings("Qt-Ros Package", "gui_lider_seguidor"); // Lee la configuración de la GUI en su ejecución pasada.
    restoreGeometry(settings.value("geometry").toByteArray()); // Restaura geometria.
    restoreState(settings.value("windowState").toByteArray()); // Restaura el estado del widget.
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString(); // Da la dirección del maestro, la segunda es la default.
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();//Da la dirección del host, la segunda es la default.
    // QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url); // Carga la dirección del maestro a la caja de texto del maestro.
    ui.line_edit_host->setText(host_url); // Carga la dirección del maestro a la caja de texto del host.
    // ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool(); // Obtiene el valor del bit remember.
    ui.checkbox_remember_settings->setChecked(remember); // Pasa el valor de remember a recordar configuraciones.
    bool checked = settings.value("use_environment_variables", false).toBool(); // Obtiene el valor del bit checked.
    ui.checkbox_use_environment->setChecked(checked); // Pasa el valor a la casilla usar ambiente.
    if ( checked ) { // Si fue usa el ambiente.
        ui.line_edit_master->setEnabled(false); // Deshabilita la caja de texto del maestro.
        ui.line_edit_host->setEnabled(false); // Deshabilita la caja de texto del Host.
        // ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {//Método para escribir configuraciones.
    QSettings settings("Qt-Ros Package", "gui_lider_seguidor"); // Crea el objeto settings.
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
    qnode.Enviar_Encendido(false); // Al iniciar envia el encendido como falso.
    for (int c = 1; c <= 32767; c++){ // Da un tiempo de espera para garantizar que se envio el dato.
          for (int d = 1; d <= 2500; d++)
          {}
    }
}

void MainWindow::closeEvent(QCloseEvent *event) // Método ejecutado cuando se cierra la ventana.
{
    WriteSettings(); // Escribe las configuraciones.
    QMainWindow::closeEvent(event); // Cierra la ventana.
}

}  // namespace gui_lider_seguidor
