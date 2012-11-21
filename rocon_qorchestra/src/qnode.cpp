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

#include <string>
#include <sstream>

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <willow_app_manager/ListApps.h>
#include <willow_app_manager/GetInstallationState.h>
#include <willow_app_manager/GetAppDetails.h>
#include <willow_app_manager/InstallApp.h>
#include <willow_app_manager/UninstallApp.h>
#include <willow_app_manager/StartApp.h>
#include <willow_app_manager/StopApp.h>

#include "../include/rocon_qorchestra/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_qorchestra {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc_(argc),
	init_argv_(argv)
{
//	QStringList concert_clients_model_header;
	QStringList model_header;
//	concert_clients_model_header << "Unique Name" << "Suggested Name" << "Device" << "Key" << "Uri" << "Last Connection" << "Status";
	model_header << "Unique Name" << "Suggested Name" << "Device" << "Key" << "Uri" << "Last Connection" << "Connected";
	concert_clients_model_.setHorizontalHeaderLabels( model_header );
//	model_header.clear();
//	model_header << "App name" << "Installed";
//	QStringList client_apps_model_header;
//	client_apps_model_header << "Unique Name" << "Suggested Name" << "App name" << "Installed";
//  client_apps_model_.setHorizontalHeaderLabels( client_apps_model_header );
//	concert_clients_apps_model_.setHorizontalHeaderLabels( model_header );
}

QNode::~QNode() {
    if( ros::isStarted() ) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()
{
	ros::init(init_argc_, init_argv_, "rocon_qorchestra");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	establishRosComms();
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url)
{
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"rocon_qorchestra");
	if ( ! ros::master::check() )
	  return false;
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	establishRosComms();
	start();
	return true;
}

void QNode::establishRosComms()
{
	ros::NodeHandle nh;
	concert_clients_subscriber_ = nh.subscribe("concert_clients",10, &QNode::subscribeConcertClients, this);
}

void QNode::run() {
	implementations.fetch();
	ros::spin();
//	ros::Rate loop_rate(10);
//	int count = 0;
//	while ( ros::ok() ) {
//
//		std_msgs::String msg;
//		std::stringstream ss;
//		ss << "hello world " << count;
//		msg.data = ss.str();
//		chatter_publisher.publish(msg);
//		log(Info,std::string("I sent: ")+msg.data);
//		ros::spinOnce();
//		loop_rate.sleep();
//		++count;
//	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


/*****************************************************************************
** Ros Comms
*****************************************************************************/
/**
 * Catches the concert client list as it gets published.
 *
 * concert_msgs/ConcertClient:
 *  zeroconf_msgs/DiscoveredService zeroconf
 *	string platform
 *	string system
 *	string suggested_name
 *	string unique_name
 *	string key
 *	string app_manager_uri
 *	bool is_connected
 *	time last_connection_timestamp
 */
void QNode::subscribeConcertClients(const concert_msgs::ConcertClientsConstPtr concert)
{
  concert_clients_ptr_ = concert;// store pointer for update
	for ( unsigned int i = 0; i < concert->clients.size(); ++i )
	{
		ROS_DEBUG_STREAM("Concert Clients: =========== Concert Client List ===========");
		ROS_DEBUG_STREAM("Concert Clients: \t unique name: " << concert->clients[i].unique_name );
		ROS_DEBUG_STREAM("Concert Clients: \t platform: " << concert->clients[i].platform << "."
		  << concert->clients[i].system << "." << concert->clients[i].robot);
		ROS_DEBUG_STREAM("Concert Clients: \t suggested_name: " << concert->clients[i].suggested_name );
		ROS_DEBUG_STREAM("Concert Clients: \t key: " << concert->clients[i].key );
		ROS_DEBUG_STREAM("Concert Clients: \t app_manager_uri: " << concert->clients[i].app_manager_uri );
		ROS_DEBUG_STREAM("Concert Clients: \t last_connection_timestamp: "
		  << concert->clients[i].last_connection_timestamp );
		if ( concert->clients[i].is_connected )
			ROS_DEBUG_STREAM("Concert Clients: \t is_connected : yes" );
		else
			ROS_DEBUG_STREAM("Concert Clients: \t is_connected : no" );
		ROS_DEBUG_STREAM("Concert Clients: \t zeroconf info: todo" );
	}

    // see QGooNode::requestStatus() for an example where it clears, then sets the size explicitly
	// blocking signals is useless here, tableview malfunctions withough them.
    // bool old_signal_state = concert_clients_model.blockSignals(true);
    // concert_clients_model.blockSignals(old_signal_state);
	mutex_.lock();
	concert_clients_model_.removeRows(0,concert_clients_model_.rowCount());
	concert_clients_apps_model_.clear();
	for ( unsigned int i = 0; i < concert->clients.size(); ++i )
	{
		QList<QStandardItem *> items;
		items << createConcertClientDataElement(concert->clients[i].unique_name);
		items << createConcertClientDataElement(concert->clients[i].suggested_name);
		items << createConcertClientDataElement(concert->clients[i].platform + "." + concert->clients[i].system + "."
		    + concert->clients[i].robot);
		items << createConcertClientDataElement(concert->clients[i].key);
		items << createConcertClientDataElement(concert->clients[i].app_manager_uri,Qt::AlignLeft);
		std::stringstream ostream;
		if ( concert->clients[i].is_connected ) {
			ostream << concert->clients[i].last_connection_timestamp;
			items << createConcertClientDataElement(ostream.str());
			items << createConcertClientDataElement("ready boss",Qt::AlignLeft);
		} else {
			items << createConcertClientDataElement("-");
			items << createConcertClientDataElement("-",Qt::AlignLeft);
		}
		concert_clients_model_.appendRow(items);
    /*
     * Get apps of the client
     */
    getClientAppsData(concert->clients[i].unique_name);
	}
    Q_EMIT concertClientsUpdate();
	mutex_.unlock();

  /*
   * Clear client's apps view
   */
  mutex_.lock();
  client_apps_model_.clear();
  QStringList model_header;
  model_header << "Installable apps" << "Installed apps";
  client_apps_model_.setHorizontalHeaderLabels( model_header );
  Q_EMIT clientAppListRetrieved( &client_apps_model_ );
  mutex_.unlock();
  /*
   * Clear app details view
   */
  mutex_.lock();
  app_details_model_.clear();
  model_header.clear();
  model_header << "";
  app_details_model_.setHorizontalHeaderLabels( model_header );
  model_header.clear();
  model_header << "Name" << "Version" << "Description";
  app_details_model_.setVerticalHeaderLabels( model_header );
  Q_EMIT appDetailsRetrieved( &app_details_model_ );
  mutex_.unlock();
}


/**
 * Given a robot reference in the model, this retrieves the current apps as well
 * as the apps in the store.
 */
void QNode::getClientAppsData(std::string robot_namespace)
{
//  std::string robot_namespace = concert->clients[row].unique_name;
  ros::NodeHandle nh;
  std::stringstream ostream;
  ostream << robot_namespace << "'s apps";
  QStandardItem* client_apps = new QStandardItem( QString(ostream.str().c_str()) );
  /*
   * Get client's installable apps
   */
  QList<QStandardItem*> installable_apps;
  ros::ServiceClient list_exchange_apps_client = nh.serviceClient<willow_app_manager::GetInstallationState>(
      "/" + robot_namespace + "/list_exchange_apps");
  willow_app_manager::GetInstallationState list_installable_apps;
  if ( list_exchange_apps_client.waitForExistence( ros::Duration(1.0)) )
  {
    list_installable_apps.request.remote_update = true;
    if (list_exchange_apps_client.call(list_installable_apps))
    {
      for(unsigned int j = 0; j < list_installable_apps.response.available_apps.size(); ++j)
      {
        installable_apps << new QStandardItem(list_installable_apps.response.available_apps[j].name.c_str());
        ROS_INFO_STREAM("Added app " << list_installable_apps.response.available_apps[j].name << " to the list of "
          << robot_namespace << "'s installable apps.");
      }
    }
    else
    {
      ROS_WARN_STREAM("Service call failed, when trying to receive installable apps list from "
          << robot_namespace << ".");
      installable_apps << createConcertClientDataElement("No installable apps have been retrieved.");
    }
  }
  else
  {
    ROS_WARN_STREAM("Service for retrieving " << robot_namespace << "'s installable apps was not advertised in time.");
    installable_apps << createConcertClientDataElement("No installable apps have been retrieved.");
  }

  /*
   * Get client's installed apps
   */
  ros::ServiceClient list_apps_client = nh.serviceClient<willow_app_manager::ListApps>("/" + robot_namespace +
      "/list_apps");
  willow_app_manager::ListApps list_installed_apps;
  QList<QStandardItem*> installed_apps;
  if ( list_apps_client.waitForExistence( ros::Duration(1.0)) )
  {
    if (list_apps_client.call(list_installed_apps))
    {
      for(unsigned int j = 0; j < list_installed_apps.response.available_apps.size(); ++j)
      {
        installed_apps << new QStandardItem(list_installed_apps.response.available_apps[j].name.c_str());
        ROS_INFO_STREAM("Added app " << list_installed_apps.response.available_apps[j].name << " to the list of "
          << robot_namespace << "'s installed apps.");
      }
    }
    else
    {
      ROS_WARN_STREAM("Service call failed, when trying to receive installed apps list from " << robot_namespace << ".");
      installed_apps << createConcertClientDataElement("No installed apps have been retrieved.");
    }
  }
  else
  {
    ROS_WARN_STREAM("Service for retrieving " << robot_namespace << "'s apps was not advertised in time.");
    installed_apps << createConcertClientDataElement("No installed apps have been retrieved.");
  }
//  mutex_.lock();
  client_apps->appendColumn(installable_apps);
  client_apps->appendColumn(installed_apps);
  concert_clients_apps_model_.appendRow(client_apps);
//  mutex_.unlock();
}


void QNode::retrieveClientAppList( const QModelIndex &index )
{
  current_client_index_ = index;
  ROS_INFO_STREAM("Retrieving apps of client '"
    << concert_clients_ptr_->clients[current_client_index_.row()].unique_name << "'.");
  client_installable_apps_list_ << createConcertClientAppsDataElement("nothing here ...", Qt::AlignLeft);
  client_installed_apps_list_ << createConcertClientAppsDataElement("nothing here ...", Qt::AlignLeft);

  if ( index.row() >= 0 && index.row() < concert_clients_apps_model_.rowCount() )
  {
    int row_count = concert_clients_apps_model_.item(index.row())->rowCount();
//    int column_count = concert_clients_apps_model_.item(row)->columnCount();
//    ROS_INFO_STREAM("Client in row " << row << ": item details: rows =" << row_count
//        << " columns = " << column_count);

    client_installable_apps_list_.clear();
    for ( int i = 0; i < row_count; ++i )
      if ( concert_clients_apps_model_.item(index.row())->child(i,0) )
        client_installable_apps_list_ << createConcertClientAppsDataElement(
          concert_clients_apps_model_.item(index.row())->child(i,0)->text().toStdString(), Qt::AlignLeft);

    client_installed_apps_list_.clear();
    for ( int i = 0; i < row_count; ++i )
      if ( concert_clients_apps_model_.item(index.row())->child(i,1) )
        client_installed_apps_list_ << createConcertClientAppsDataElement(
          concert_clients_apps_model_.item(index.row())->child(i,1)->text().toStdString(), Qt::AlignLeft);
  }
  else
    ROS_ERROR_STREAM("No client data exists in row number " << index.row() << "! (row count "
      << client_apps_model_.rowCount() << ")");

//  ROS_INFO_STREAM("client_app_list size = " << client_app_list.size());
//  for ( int i = 0; i < client_app_list.size(); ++i )
//    ROS_INFO_STREAM("Client's app list: item " << i << ": " << client_app_list.at(i)->text().toStdString());
//  ROS_INFO_STREAM("client_installed_apps_list size = " << client_installed_apps_list.size());
//  for ( int i = 0; i < client_installed_apps_list.size(); ++i )
//    ROS_INFO_STREAM("Client's installed apps list: item " << i << ": " << client_installed_apps_list.at(i)->text().toStdString());

  mutex_.lock();
  client_apps_model_.clear();
  client_apps_model_.appendColumn(client_installable_apps_list_);
  client_apps_model_.appendColumn(client_installed_apps_list_);
  QStringList model_header;
  model_header << "Installable apps" << "Installed apps";
  client_apps_model_.setHorizontalHeaderLabels( model_header );
  mutex_.unlock();
//  ROS_INFO_STREAM("client_apps_model_ size: rows = " << client_apps_model_.rowCount() << ", columns = "
//    << client_apps_model_.columnCount());

  Q_EMIT clientAppListRetrieved(&client_apps_model_);
}


void QNode::retrieveAppDetails( const QModelIndex &index )
{
  ros::NodeHandle nh;
  std::string robot_namespace = concert_clients_ptr_->clients[current_client_index_.row()].unique_name;
  ros::ServiceClient app_details_client = nh.serviceClient<willow_app_manager::GetAppDetails>(
        "/" + robot_namespace + "/get_app_details");
  willow_app_manager::GetAppDetails app_details;
  app_details_model_.appendRow(createConcertClientAppsDataElement("nothing here ...", Qt::AlignLeft));
  QList<QStandardItem*> app_details_list;
  QStringList model_hor_header;
  QStringList model_ver_header;
  model_hor_header << "";
  model_ver_header << "";
  if ( index.column() == 0 )
    app_details.request.name = client_installable_apps_list_.at(index.row())->text().toStdString();
  else
    app_details.request.name = client_installed_apps_list_.at(index.row())->text().toStdString();
  ROS_INFO_STREAM("Requesting details of app '" << app_details.request.name << "' on '"
    << robot_namespace << "'.");
  if (app_details_client.call(app_details) )
  {
    ROS_INFO_STREAM("Service call to get details of app '" << app_details.request.name << "' was successful.");
    app_details_list << createConcertClientAppsDataElement(app_details.response.app.display_name, Qt::AlignLeft);
//    app_details_list << new QStandardItem( QIcon(QPixmap(app_details.response.app.icon.data.)),
//      QString(app_details.response.app.display_name.c_str() + "_icon") );
    app_details_list << createConcertClientAppsDataElement(app_details.response.app.latest_version, Qt::AlignLeft);
    app_details_list << createConcertClientAppsDataElement(app_details.response.app.description, Qt::AlignLeft);
    model_hor_header.clear();
    model_ver_header.clear();
    model_hor_header << QString(app_details.response.app.name.c_str());
    model_ver_header << "Name" << "Version" << "Description";
  }
  else
    ROS_WARN_STREAM("Service call to get details of app '" << app_details.request.name << "' was not successful!");

  mutex_.lock();
  app_details_model_.clear();
  app_details_model_.appendColumn(app_details_list);
  app_details_model_.setHorizontalHeaderLabels( model_hor_header );
  app_details_model_.setVerticalHeaderLabels( model_ver_header );
  mutex_.unlock();
  Q_EMIT appDetailsRetrieved(&app_details_model_);
}


void QNode::installApp( const QModelIndex &index )
{
  ros::NodeHandle nh;
  std::string robot_namespace = concert_clients_ptr_->clients[current_client_index_.row()].unique_name;
  ros::ServiceClient install_app_client = nh.serviceClient<willow_app_manager::InstallApp>(
        "/" + robot_namespace + "/install_app");
  willow_app_manager::InstallApp install_app;
  install_app.request.name = client_installable_apps_list_.at(index.row())->text().toStdString();
  ROS_INFO_STREAM("Requesting installation of app '" << install_app.request.name << "' on '"
    << robot_namespace << "'.");
  if (install_app_client.call(install_app) )
    ROS_INFO_STREAM("Service call to install app '" << install_app.request.name << "' was successful.");
  else
    ROS_WARN_STREAM("Service call to install app '" << install_app.request.name << "' was not successful!");

  // to refresh the app list; TODO: try to load (only) a fresh app list
  subscribeConcertClients(concert_clients_ptr_);
//  retrieveClientAppList(current_client_index_);
}


void QNode::uninstallApp( const QModelIndex &index )
{
  ros::NodeHandle nh;
  std::string robot_namespace = concert_clients_ptr_->clients[current_client_index_.row()].unique_name;
  ros::ServiceClient uninstall_app_client = nh.serviceClient<willow_app_manager::UninstallApp>(
        "/" + robot_namespace + "/uninstall_app");
  willow_app_manager::UninstallApp uninstall_app;
  uninstall_app.request.name = client_installed_apps_list_.at(index.row())->text().toStdString();
  ROS_INFO_STREAM("Requesting uninstallation of app '" << uninstall_app.request.name << "' on '"
      << robot_namespace << "'.");
  if (uninstall_app_client.call(uninstall_app) )
    ROS_INFO_STREAM("Service call to uninstall app '" << uninstall_app.request.name << "' was successful.");
  else
    ROS_WARN_STREAM("Service call to uninstall app '" << uninstall_app.request.name << "' was not successful!");

  // to refresh the app list; TODO: try to load (only) a fresh app list
  subscribeConcertClients(concert_clients_ptr_);
//  retrieveClientAppList( current_client_index_ );
}

/**
 * Checks to see if the client is connected.
 *
 * This is not thread safe, make sure it's used inside a mutex lock/unlock.
 */
bool QNode::clientIsConnected(const int &row) {
	if ( concert_clients_model_.item(row, 6)->text().toStdString() == "ready boss" ) {
		return true;
	} else {
		return false;
	}
}
/**
 * Checks to see if the client is android (bad hack till we can remove it).
 *
 * This is not thread safe, make sure it's used inside a mutex lock/unlock.
 */
bool QNode::clientIsAndroid(const int &row) {
	if ( concert_clients_model_.item(row, 2)->text().toStdString() == "android.ros.xoom" ) {
		return true;
	} else {
		return false;
	}
}
/**
 * Returns the concert client's device name (triple).
 *
 * This is not thread safe, make sure it's used inside a mutex lock/unlock.
 */
QString QNode::clientDeviceTriple(const int &row) {
	return concert_clients_model_.item(row, 2)->text();
}

/**
 * Compare the listed device configurations with the available concert clients and their installed apps.
 * If all necessary configurations are available, allow "Start solutions".
 * Otherwise,
 * if not enough platforms are available, inform the user to wait or
 * if not all apps are installed, ask for the confirmation to install the missing apps.
 * Afterwards allow "Start solution"
  */
void QNode::checkSolution(const QModelIndex &index) {

	mutex_.lock();
	QModelIndex group_index;
	QModelIndex parent = implementations.model()->parent(index);
	if ( parent.isValid() ) {
		group_index = parent;
	} else {
		group_index = index;
	}
	QStringList requested_devices, requested_device_configurations;
	for ( int i = 0; i < implementations.model()->rowCount(group_index); ++i ) {
		QModelIndex child = group_index.child(i,0);
		if ( child.isValid() ) {
			std::string device_configuration(implementations.model()->itemFromIndex(child)->text().toStdString());
			requested_device_configurations << device_configuration.c_str();
			std::string only_device = device_configuration;
			only_device.resize(only_device.find_last_of("."));
			if ( !requested_devices.contains(only_device.c_str()) ) {
				requested_devices << only_device.c_str();
			}
			ROS_INFO_STREAM("QOrchestra: added device configuration: " << only_device << " [" << device_configuration << "]");
		}
	}

	// creating all available device configurations (platform + installed apps)
	QStringList available_devices, available_device_configs;
	for(int i = 0; i < concert_clients_model_.rowCount(); ++i) {
		// first check, if the concert client is connected at all
		if ( clientIsConnected(i) && !clientIsAndroid(i) ) {
			QString device = clientDeviceTriple(i);
			available_devices << device;
			ROS_DEBUG_STREAM("QOrchestra: Added available device: " << device.toStdString());
			for(int j = 0; j < concert_clients_apps_model_.item(i)->rowCount(); ++j) {
				if (concert_clients_apps_model_.item(i)->child(j,1) ) {
					std::stringstream stream;
					stream << device.toStdString() << "." << concert_clients_apps_model_.item(i)->child(j,1)->text().toStdString();
					available_device_configs << QString(stream.str().c_str());
					ROS_DEBUG_STREAM("QOrchestra: Added available device configuration: " << stream.str());
				}
			}
		} else if ( clientIsConnected(i) && clientIsAndroid(i) ) { // ugly hack
			QString device( concert_clients_model_.item(i, 2)->text() ); // get the detailed client name
			available_devices << device;
			ROS_DEBUG_STREAM("QOrchestra: Added available device: " << device.toStdString());
			std::stringstream stream;
			stream << device.toStdString() << "." << "rocon_voice_commander";
			available_device_configs << QString(stream.str().c_str());
			ROS_DEBUG_STREAM("QOrchestra: Added available device configuration: " << stream.str());
		}
	}
	mutex_.unlock();

  // checks requested against available devices and device configurations
  QStringList missing_devices(requested_devices);
  QStringList missing_device_configurations(requested_device_configurations);
  for(int i = 0; i < requested_device_configurations.size(); ++i)
  {
    if ( available_device_configs.contains(requested_device_configurations.at(i)) )
    {
      missing_device_configurations.removeOne(requested_device_configurations.at(i));
      ROS_INFO_STREAM("QOrchestra: Requested device configuration "
        << requested_device_configurations.at(i).toStdString() << " is available.");
    }
  }
  for(int i = 0; i < requested_devices.size(); ++i)
  {
    if ( available_devices.contains(requested_devices.at(i)) )
    {
      missing_devices.removeOne(requested_devices.at(i));
      ROS_INFO_STREAM("QOrchestra: Requested device " << requested_devices.at(i).toStdString() << " is available.");
    }
  }

  // checks which requirements are met
  if ( missing_device_configurations.size() == 0 )
  {
    ROS_INFO_STREAM("QOrchestra: All device configurations for the selected solution are available.");
    ROS_INFO_STREAM("QOrchestra: You can start the solution now.");
    solution_device_configurations_ = requested_device_configurations;
    Q_EMIT solutionRequirementsMet();
  }
  else if ( missing_devices.size() == 0 )
  {
    ROS_INFO_STREAM("QOrchestra: Some device configurations are missing, but all requested devices are available.");
    QStandardItem* missing_apps_on_devices = new QStandardItem("missing_apps_on_devices");
    for(int i = 0; i < missing_device_configurations.size(); ++i)
    {
      ROS_INFO_STREAM("QOrchestra: Checking missing device configuration '"
        << missing_device_configurations.at(i).toStdString());
      QString device_str = missing_device_configurations.at(i);
      device_str.resize(device_str.lastIndexOf("."));
      QStandardItem* device = new QStandardItem(device_str);
      QString app_str = missing_device_configurations.at(i);
      app_str.remove(0, app_str.lastIndexOf(".") + 1);
      QStandardItem* app = new QStandardItem(app_str);
      QList<QStandardItem*> device_and_app_items;
      device_and_app_items << device;
      device_and_app_items << app;
      missing_apps_on_devices->appendRow(device_and_app_items);
      ROS_INFO_STREAM("QOrchestra: Solution is missing app '"
        << missing_apps_on_devices->child(i, 1)->text().toStdString() << "' on device '"
        << missing_apps_on_devices->child(i, 0)->text().toStdString() << "'.");
    }

    ROS_INFO_STREAM("QOrchestra: Proceeding with asking for permission to install missing apps.");
    Q_EMIT installMissingAppsRequest( missing_apps_on_devices );
  }
  else
  {
    ROS_WARN_STREAM("QOrchestra: Not all device have connected to the concert master yet!");
    ROS_WARN_STREAM("QOrchestra: Please wait until they connect and check the solution again.");
  }
}


void QNode::installMissingApps( const QStandardItem* missing_apps_on_devices )
{
  ROS_INFO_STREAM("QOrchestra: Installing the missing apps on the corresponding devices.");
  ros::NodeHandle nh;
  int missing_apps_installed = 0;
  for( int i = 0; i < missing_apps_on_devices->rowCount(); ++i )
  {
    // search connected clients for corresponding platforms and retrieve their name
    for( int j = 0; j < concert_clients_model_.rowCount(); ++j )
    {
      if ( missing_apps_on_devices->child(i, 0)->text().toStdString() ==
          concert_clients_model_.item(j, 2)->text().toStdString() )
      {
        std::string robot_namespace = concert_clients_model_.item(j, 0)->text().toStdString();
        ros::ServiceClient install_app_client = nh.serviceClient<willow_app_manager::InstallApp>(
          "/" + robot_namespace + "/install_app");
        willow_app_manager::InstallApp install_app;
        install_app.request.name = missing_apps_on_devices->child(i, 1)->text().toStdString();
        ROS_INFO_STREAM("QOrchestra: Requesting installation of app '"
            << missing_apps_on_devices->child(i, 1)->text().toStdString() << "' on device '"
            << missing_apps_on_devices->child(i, 0)->text().toStdString() << "'.");
        if ( install_app_client.call(install_app) )
        {
          missing_apps_installed++;
          j = concert_clients_model_.rowCount();
          ROS_INFO_STREAM("QOrchestra: Service call to install app '"
            << missing_apps_on_devices->child(i, 1)->text().toStdString() << "' on device '"
            << missing_apps_on_devices->child(i, 0)->text().toStdString() << "' was successful.");
        }
        else
          ROS_WARN_STREAM("QOrchestra: Could not install app '"
            << missing_apps_on_devices->child(i, 1)->text().toStdString() << "' on device '"
            << missing_apps_on_devices->child(i, 0)->text().toStdString() << "'.");
      }
    }
  }
  if (missing_apps_installed == missing_apps_on_devices->rowCount() )
  {
    ROS_INFO_STREAM("QOrchestra: All missing apps have been installed!");
    ROS_INFO_STREAM("QOrchestra: Checking the solution again.");
    if ( concert_clients_ptr_ )
    {
        subscribeConcertClients(concert_clients_ptr_);
        ros::Duration(2.0).sleep(); // wait some time to let the qorchestra refresh the apps lists of the client
        // TODO: improve this!
        Q_EMIT missingAppsInstalled(check_solution_index_);
    }
    else
    {
      ROS_WARN_STREAM("QOrchestra: Strange, seems like no concert clients have been found yet!");
      ROS_WARN_STREAM("QOrchestra: Please manually confirm the solution again in order to ensure, that all necessary"
          << "clients are still connected and apps installed.");
    }
  }
  else
  {
    ROS_WARN_STREAM("QOrchestra: Not all missing apps could be installed!");
    ROS_WARN_STREAM("QOrchestra: Please run the confirmation of the selected solution again.");
  }
}

void QNode::triggerSolution(bool start) {
    ROS_INFO_STREAM("QOrchestra: Preparing for starting the solution's apps.");
    if ( solution_device_configurations_.size() !=  0 ) {
    	QStandardItem* solution = new QStandardItem("solution");

		// separate apps from device and search for the clients corresponding to the devices
		for( int i = 0; i < solution_device_configurations_.size(); ++i ) {
			if ( concert_clients_model_.rowCount() != 0 ) {
				QString device_str = solution_device_configurations_.at(i);
				device_str.resize(device_str.lastIndexOf("."));
				QStandardItem* device = new QStandardItem(device_str);
				QString app_str = solution_device_configurations_.at(i);
				app_str.remove(0, app_str.lastIndexOf(".") + 1);
				QStandardItem* app = new QStandardItem(app_str);
				QStandardItem* client_unique_name = new QStandardItem("empty");
				QString solution_device = solution_device_configurations_.at(i);
				solution_device.resize(solution_device_configurations_.at(i).lastIndexOf("."));

				for( int j = 0; j < concert_clients_model_.rowCount(); ++j ) {
					if ( concert_clients_model_.item(j, 2)->text() == solution_device ) {
						QString client_unique_name_str = concert_clients_model_.item(j, 0)->text();
						client_unique_name = new QStandardItem(client_unique_name_str);
						break;
					}
				}
				QList<QStandardItem*> names_devices_apps;
				names_devices_apps << client_unique_name << device << app;
				solution->appendRow(names_devices_apps);
			} else {
				ROS_ERROR_STREAM("QOrchestra: no concert clients connected yet!");
				return;
			}
		}

		// trigger the service calls to start the apps
		ros::NodeHandle nh;
		int apps_triggered = 0;
		for( int i = 0; i < solution->rowCount(); ++i ) {
//      if ( missing_apps_on_devices->child(i, 0)->text().toStdString() ==
//                concert_clients_model_.item(j, 2)->text().toStdString() )
            {
            	std::string robot_namespace = solution->child(i, 0)->text().toStdString();
            	std::string app_name = solution->child(i, 2)->text().toStdString();
            	std::string trigger_string;
            	if ( start ) {
            		ros::ServiceClient app_client = nh.serviceClient<willow_app_manager::StartApp>("/" + robot_namespace + "/start_app");
            		willow_app_manager::StartApp start_app;
            		start_app.request.name = app_name;
            		if ( app_client.call(start_app) ) {
            			apps_triggered++;
            			ROS_INFO_STREAM("QOrchestra: service call to start app was successful [" << app_name << "][" << robot_namespace << "]");
            		} else {
            			ROS_WARN_STREAM("QOrchestra: service call to start app failed [" << app_name << "][" << robot_namespace << "]");
            		}
            	} else {
            		ros::ServiceClient app_client = nh.serviceClient<willow_app_manager::StopApp>("/" + robot_namespace + "/stop_app");
            		willow_app_manager::StopApp stop_app;
            		stop_app.request.name = app_name;
            		if ( app_client.call(stop_app) ) {
            			apps_triggered++;
            			ROS_INFO_STREAM("QOrchestra: service call to stop app was successful [" << app_name << "][" << robot_namespace << "]");
            		} else {
            			ROS_WARN_STREAM("QOrchestra: service call to stop app failed [" << app_name << "][" << robot_namespace << "]");
            		}
            	}
            }
		}
		if ( apps_triggered == solution->rowCount() ) {
        	if ( start ) {
    			ROS_INFO_STREAM("QOrchestra: all apps started.");
        	} else {
    			ROS_INFO_STREAM("QOrchestra: all apps stopped.");
        	}
		} else {
        	if ( start ) {
    			ROS_ERROR_STREAM("QOrchestra: not all apps started.");
        	} else {
    			ROS_ERROR_STREAM("QOrchestra: not all apps stopped.");
        	}
		}
    } else {
        ROS_ERROR_STREAM("QOrchestra: solution contains no apps to start!");
    }
}


/*****************************************************************************
** Utilities
*****************************************************************************/

QStandardItem* QNode::createConcertClientDataElement(const std::string &str, const Qt::Alignment &alignment)
{
	QStandardItem *item = new QStandardItem( QString(str.c_str()) );
	item->setTextAlignment(alignment);
	item->setEditable(false); // some of these will be editable later (to send commands).
	return item;
}


QStandardItem* QNode::createConcertClientAppsDataElement(const std::string &str, const Qt::Alignment &alignment)
{
  QStandardItem *item = new QStandardItem( QString(str.c_str()) );
  item->setTextAlignment(alignment);
  item->setEditable(false); // some of these will be editable later (to send commands).
  //item->setEnabled(false); // some of these will be editable later (to send commands).
  return item;
}

}  // namespace rocon_qorchestra
