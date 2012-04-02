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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rocon_qorchestra/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace rocon_qorchestra
{

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), qnode(argc, argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp,
    SLOT(aboutQt())); // qApp is a global variable for the application

	ReadSettings();
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
	** Concert Clients
	**********************/
    QObject::connect(&qnode, SIGNAL(concertClientsUpdate()), this, SLOT(updateConcertClientsViews()));
	QHeaderView *header = new QHeaderView(Qt::Horizontal);
	header->setResizeMode(QHeaderView::ResizeToContents);
	header->setStretchLastSection(true);
	ui.table_view_concert_clients->setHorizontalHeader(header);
    ui.table_view_concert_clients->setModel(qnode.concertClientsModel());
    ui.tree_view_implementations->setModel(qnode.implementations.model());

    /*********************
	** Sigslots
	**********************/
    QObject::connect(this, SIGNAL(tableViewConcertClientsClicked( const QModelIndex )),
      &qnode, SLOT(retrieveClientAppList( const QModelIndex )));
    QObject::connect(&qnode, SIGNAL(clientAppListRetrieved( QStandardItemModel* )),
      this, SLOT(updateClientAppsViews( QStandardItemModel* )));

    QObject::connect(this, SIGNAL(tableViewClientAppsClicked( const QModelIndex )),
      &qnode, SLOT(retrieveAppDetails( const QModelIndex )));
    QObject::connect(&qnode, SIGNAL(appDetailsRetrieved( QStandardItemModel* )),
      this, SLOT(updateAppDetailsViews( QStandardItemModel* )));

    QObject::connect(&(qnode.implementations), SIGNAL(sigImplementationsFetched()), this, SLOT(updateViewImplementations()));
    QObject::connect(&qnode, SIGNAL(installMissingAppsRequest ( const QStandardItem* )),
      this, SLOT(installMissingAppsBox( const QStandardItem* )));
    QObject::connect(this, SIGNAL(installMissingAppsConfirmed( const QStandardItem* )),
        &qnode, SLOT(installMissingApps( const QStandardItem* )));
    QObject::connect(&qnode, SIGNAL(solutionRequirementsMet()),
      this, SLOT(enableStartSolution()));

    QObject::connect(this, SIGNAL(buttonInstallAppClicked(const QModelIndex)),
      &qnode, SLOT(installApp(const QModelIndex)));
    QObject::connect(this, SIGNAL(buttonUninstallAppClicked(const QModelIndex)),
      &qnode, SLOT(uninstallApp(const QModelIndex)));
    QObject::connect(&qnode, SIGNAL(missingAppsInstalled( const QModelIndex )),
      &qnode, SLOT(checkSolution( const QModelIndex )));

	/*********************
	 ** Auto Start
	 **********************/
	if (ui.checkbox_remember_settings->isChecked())
		on_button_connect_clicked(true);
}


MainWindow::~MainWindow()
{
}


/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::ReadSettings()
{
  QSettings settings("Qt-Ros Package", "rocon_qorchestra");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  QString master_url = settings.value("master_url",
      QString("http://192.168.1.2:11311/")).toString();
  QString host_url =
      settings.value("host_url", QString("192.168.1.3")).toString();
  //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
  ui.line_edit_master->setText(master_url);
  ui.line_edit_host->setText(host_url);
  //ui.line_edit_topic->setText(topic_name);
  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if (checked) {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
    //ui.line_edit_topic->setEnabled(false);
  }
}

void MainWindow::WriteSettings()
{
  QSettings settings("Qt-Ros Package", "rocon_qorchestra");
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  //settings.setValue("topic_name",ui.line_edit_topic->text());
  settings.setValue("use_environment_variables",
  QVariant(ui.checkbox_use_environment->isChecked()));
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
  settings.setValue("remember_settings",
  QVariant(ui.checkbox_remember_settings->isChecked()));
}


void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QMainWindow::closeEvent(event);
}


void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}


/*****************************************************************************
 ** Implementation [Slots][automaticcally connected]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(
      this,
      tr("About ..."),
      tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}


/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */
void MainWindow::on_button_connect_clicked(bool check)
{
	if (ui.checkbox_use_environment->isChecked()) {
		if (!qnode.init()) {
		  showNoMasterMessage();
		} else {
		  ui.button_connect->setEnabled(false);
		  ui.toolbox_command->setCurrentIndex(1);
		}
	} else {
	    if (!qnode.init(ui.line_edit_master->text().toStdString(), ui.line_edit_host->text().toStdString())) {
	        showNoMasterMessage();
	    } else {
		    ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		    ui.toolbox_command->setCurrentIndex(1);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
	bool enabled;
	if (state == 0)
		enabled = true;
	else
		enabled = false;
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}


void MainWindow::on_table_view_concert_clients_clicked(const QModelIndex &index)
{
//  std::cout << "Index clicked: " << index.row() << "," << index.column() << std::endl;
  Q_EMIT tableViewConcertClientsClicked( index );
}


void MainWindow::on_table_view_client_apps_clicked(const QModelIndex &index)
{
  std::cout << "Index clicked: " << index.row() << "," << index.column() << std::endl;
  client_app_list_index_ = index;
  Q_EMIT tableViewClientAppsClicked( index );
}



void MainWindow::on_button_update_concert_clients_clicked()
{
  Q_EMIT buttonUpdateConcertClientsClicked();
}


void MainWindow::on_button_install_app_clicked()
{
  if ( client_app_list_index_.column() == 0 )
    Q_EMIT buttonInstallAppClicked( client_app_list_index_ );
  else
    ROS_WARN("QOrchestra: uhm, installing an installed app does not make a lot of sense!");
}

void MainWindow::on_button_uninstall_app_clicked() {
    if ( client_app_list_index_.column() == 1 ) {
        Q_EMIT buttonUninstallAppClicked( client_app_list_index_ );
    } else {
	    ROS_WARN("QOrchestra: hm, uninstalling a not installed app does not make a lot of sense!");
    }
}

void MainWindow::on_button_check_implementation_clicked() {

	// selection mode is SingleSelection, so this should only return one
	QModelIndexList list = ui.tree_view_implementations->selectionModel()->selectedIndexes();
	if ( list.size() == 1 ) {
		qnode.checkSolution(list.at(0));
	} else { // otherwise zero
		ROS_ERROR_STREAM("QOrchestra : implementation list returned a size of zero, this is a bug in qorchestra.");
	}
}

void MainWindow::on_button_start_implementation_clicked() {
	qnode.startSolution();
	ui.button_start_implementation->setEnabled(false);
	ui.button_stop_implementation->setEnabled(true);
}

void MainWindow::on_button_stop_implementation_clicked() {
	qnode.stopSolution();
	ui.button_start_implementation->setEnabled(true);
	ui.button_stop_implementation->setEnabled(false);
}

void MainWindow::on_button_view_implementation_clicked() {
	qnode.implementations.viewLinkGraph();
}


/*****************************************************************************
 ** Implementation [Slots][manually connected]
 *****************************************************************************/

/**
 * When the model data has changed, it will emit a signal which triggers this slot.
 * This lets us fiddle with the display nicely.
 */
void MainWindow::updateConcertClientsViews() {
	ui.table_view_concert_clients->resizeRowsToContents();
	ui.table_view_concert_clients->resizeColumnsToContents();
}

void MainWindow::updateClientAppsViews( QStandardItemModel* client_apps_model)
{
//  QHeaderView *header = new QHeaderView(Qt::Horizontal);
//  header->setResizeMode(QHeaderView::ResizeToContents);
//  header->setStretchLastSection(true);
//  ui.table_view_client_apps->setHorizontalHeader(header);
  ui.table_view_client_apps->setModel(client_apps_model);
  ui.table_view_client_apps->resizeRowsToContents();
  ui.table_view_client_apps->resizeColumnsToContents();
}


void MainWindow::updateAppDetailsViews( QStandardItemModel* app_details_model)
{
  ui.table_view_app_details->setModel(app_details_model);
  ui.table_view_app_details->resizeRowsToContents();
  ui.table_view_app_details->resizeColumnsToContents();
}

/**
 * This is called when the implementations are successfully retrieved.
 * Update the view, select the first row and enable some of the buttons.
 */
void MainWindow::updateViewImplementations() {
	if ( qnode.implementations.model()->rowCount() > 0 ) {
		QModelIndex index = qnode.implementations.model()->index(0,0);
		ui.tree_view_implementations->expand(index);
		ui.tree_view_implementations->setCurrentIndex(index);
		ui.button_check_implementation->setEnabled(true);
		ui.button_start_implementation->setEnabled(false);
		ui.button_view_implementation->setEnabled(true);
	}
}


void MainWindow::installMissingAppsBox( const QStandardItem* missing_apps_on_devices )
{
  QMessageBox msg_box;
  msg_box.setIcon(QMessageBox::Question);
  msg_box.setWindowTitle("Installation of missing apps");
  msg_box.addButton(QMessageBox::Ok);
  msg_box.addButton(QMessageBox::Abort);
  msg_box.setDefaultButton(QMessageBox::Ok);
  std::stringstream stream;
  stream << "The following devices are missing the listed apps:\n";
  for(int i = 0; i < missing_apps_on_devices->rowCount(); ++i)
  {
    stream << "Device '" << missing_apps_on_devices->child(i, 0)->text().toStdString() << "' is missing app '"
      << missing_apps_on_devices->child(i, 1)->text().toStdString() << "'.\n";
  }
  stream << "\nPress 'OK' to start the installation of the missing devices or 'Abort' to not install any apps.\n";
  msg_box.setText(stream.str().c_str());
  if ( msg_box.exec() == QMessageBox::Ok )
  {
    std::cout << "Confirmation to install missing apps given." << std::endl;
    Q_EMIT installMissingAppsConfirmed(missing_apps_on_devices);
  }
}


void MainWindow::enableStartSolution()
{
  ui.button_start_implementation->setEnabled(true);
}


} // namespace rocon_qorchestra

