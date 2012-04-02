/**
 * @file /include/rocon_qorchestra/main_window.hpp
 *
 * @brief Qt based gui for rocon_qorchestra.
 *
 * @date November 2010
 **/
#ifndef rocon_qorchestra_MAIN_WINDOW_H
#define rocon_qorchestra_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rocon_qorchestra {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
	void on_table_view_concert_clients_clicked(const QModelIndex &index);
	void on_table_view_client_apps_clicked(const QModelIndex &index);
	void on_button_update_concert_clients_clicked();
	void on_button_install_app_clicked();
	void on_button_uninstall_app_clicked();
	void on_button_check_implementation_clicked();
	void on_button_start_implementation_clicked();
	void on_button_stop_implementation_clicked();
	void on_button_view_implementation_clicked();

	/******************************************
	** Manual connections
	*******************************************/
	void updateConcertClientsViews();
	void updateClientAppsViews( QStandardItemModel* );
	void updateAppDetailsViews( QStandardItemModel* );
	void updateViewImplementations();
	void installMissingAppsBox( const QStandardItem* );
	void enableStartSolution();

Q_SIGNALS:
	void tableViewConcertClientsClicked( const QModelIndex &index );
	void tableViewClientAppsClicked( const QModelIndex &index );
	//void tableViewSolutionsClicked( const QModelIndex &index );
	void buttonUpdateConcertClientsClicked();
	void buttonInstallAppClicked( const QModelIndex &index );
	void buttonUninstallAppClicked( const QModelIndex &index );
	void installMissingAppsConfirmed( const QStandardItem* );
	void startSolution();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	QModelIndex client_app_list_index_;
};

}  // namespace rocon_qorchestra

#endif // rocon_qorchestra_MAIN_WINDOW_H
