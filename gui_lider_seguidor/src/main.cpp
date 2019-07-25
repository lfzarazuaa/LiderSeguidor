/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>//Libreria de Qt Main Window
#include <QApplication>//Libreria para qt gui
#include "../include/gui_lider_seguidor/main_window.hpp"//Incluir archivo generado por ui.

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);//Hacer objeto app para dejar main window.
    gui_lider_seguidor::MainWindow w(argc,argv);//Crear objeto MainWindow del namespace gui_lider_seguidor.
    w.show();//Mostrar visualmente la ventana w.
    //w2.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    //conecta la señal lastWindowClosed() a cerrar ventana para solo tener una ventana del mismo tipo.
    int result = app.exec();//Loop infinito con condicion al cerrar la ventana.
	return result;
}
