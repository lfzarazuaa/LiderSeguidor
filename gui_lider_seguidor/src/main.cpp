/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date Mayo 2019
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui> // Librería de Qt Main Window.
#include <QApplication> // Librería para interfaz gráfica.
#include "../include/gui_lider_seguidor/main_window.hpp" // Incluir archivo generado con la interfaz gráfica.

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv); // Crear objeto app para dejar creado el main window.
    gui_lider_seguidor::MainWindow w(argc,argv); // Crear objeto MainWindow del namespace gui_lider_seguidor.
    w.show(); // Mostrar visualmente la ventana w.
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    // Conecta la señal lastWindowClosed() a la función quit para cerrar la ventana.
    int result = app.exec(); // Ciclo infinito con condición al cerrar la ventana (cuando se cierra la ventana result es 0).
    return result;
}
