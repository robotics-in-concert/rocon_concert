/**
 * @file /include/rocon_qorchestra/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rocon_qorchestra_QNODE_HPP_
#define rocon_qorchestra_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QMutex>
#include <QStandardItemModel>
#include <concert_msgs/ConcertClients.h>
#include "implementations.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_qorchestra
{

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
  public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

    /*********************
    ** Models
    **********************/
    QStandardItemModel* concertClientsModel() { return &concert_clients_model_; }
    void checkSolution( const QModelIndex &index );

    /*********************
	** Variables
	**********************/
    Implementations implementations;

  public Q_SLOTS:
    void retrieveClientAppList( const QModelIndex &index );
    void retrieveAppDetails( const QModelIndex &index );
    void installApp( const QModelIndex &index );
    void uninstallApp( const QModelIndex &index );
    void installMissingApps( const QStandardItem* );
    void startSolution() { triggerSolution(true); }
    void stopSolution() { triggerSolution(false); }

  Q_SIGNALS:
    void rosShutdown();
    void concertClientsUpdate();
    void clientAppListRetrieved( QStandardItemModel* );
    void appDetailsRetrieved( QStandardItemModel* );
    void installMissingAppsRequest ( const QStandardItem* );
    void missingAppsInstalled( const QModelIndex &index );
    void solutionRequirementsMet();

  private:
    /*********************
    ** Ros Comms
    **********************/
    void establishRosComms();
    void subscribeConcertClients(const concert_msgs::ConcertClientsConstPtr concert);
    QStandardItem* createConcertClientDataElement(const std::string &str,
                                                  const Qt::Alignment &alignment = Qt::AlignHCenter);
    QStandardItem* createConcertClientAppsDataElement(const std::string &str,
                                                      const Qt::Alignment &alignment = Qt::AlignHCenter);
    void getClientAppsData(std::string robot_namespace);

    /*********************
	** Concert Client Intro
	**********************/
    bool clientIsConnected(const int &row);
    bool clientIsAndroid(const int &row);
    QString clientDeviceTriple(const int &row);

    /*********************
	** Backends
	**********************/
    void triggerSolution(bool start = true);

    int init_argc_;
    char** init_argv_;
    ros::Publisher chatter_publisher_;
    ros::Subscriber concert_clients_subscriber_;
    QStandardItemModel concert_clients_model_;
    QStandardItemModel concert_clients_apps_model_;
    QStandardItemModel client_apps_model_;
    QStandardItemModel app_details_model_;
    QMutex mutex_;
    QStringList solution_device_configurations_;

    concert_msgs::ConcertClientsConstPtr concert_clients_ptr_;
    QModelIndex current_client_index_, check_solution_index_;
    unsigned int current_highlighted_app_;
    QList<QStandardItem*> client_installable_apps_list_, client_installed_apps_list_;
};

}  // namespace rocon_qorchestra

#endif /* rocon_qorchestra_QNODE_HPP_ */
