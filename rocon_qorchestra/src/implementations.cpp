/**
 * @file /rocon_qorchestra/src/implementation.cpp
 *
 * @brief Implementation, err for the implementation!
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QTextStream>
#include <ros/ros.h>
#include <concert_msgs/Implementation.h>
#include "../include/rocon_qorchestra/implementations.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_qorchestra {

/*****************************************************************************
** Implementation
*****************************************************************************/

Implementations::~Implementations() {
	for ( unsigned int i = 0; i < external_processes.size(); ++i ) {
		if ( external_processes[i]->state() != QProcess::NotRunning ) {
			external_processes[i]->kill();
		}
		delete external_processes[i];
		external_processes[i] = NULL;
	}
}
/**
 * Currently just retrieves the orchestra loaded implementation - we may expand this
 * to cater for loading/choosing from one of multiple implementations available as well.
 */
bool Implementations::fetch() {
    ros::NodeHandle nh;
    ros::ServiceClient retreive_implementations_client = nh.serviceClient<concert_msgs::Implementation>("implementation");
    if ( retreive_implementations_client.waitForExistence( ros::Duration(5.0)) ) {
		concert_msgs::Implementation implementation;
		ROS_INFO_STREAM("QOrchestra: requesting implementations.");
		if( retreive_implementations_client.call(implementation) ) {
		    ROS_INFO_STREAM("QOrchestra: service call to load implementation was successful.");
			QList<QStandardItem*> implementation_list;
			for ( unsigned int i = 0; i < implementation.response.nodes.size(); ++i ) {
			    implementation_list << new QStandardItem(implementation.response.nodes[i].c_str());
			}
			mutex_.lock();
			dot_graph_ = implementation.response.link_graph.c_str();
			model_.clear();
			QStandardItem *parent = model_.invisibleRootItem();
			QStandardItem *header_item = new QStandardItem(implementation.response.name.c_str());
			parent->appendRow(header_item);
			header_item->appendColumn(implementation_list);
			mutex_.unlock();
			Q_EMIT sigImplementationsFetched();
			return true;
		} else {
		  ROS_WARN_STREAM("QOrchestra: service call to load implementation was not successful.");
		}
    } else {
    	ROS_WARN_STREAM("QOrchestra: implementations server unavailable (is rocon_orchestra running?)");
    }
    return false;
}

/**
 * Load up the implementation dot graph in a KGraphViewer.
 *
 * Be nice to have this internal instead of external (there are widgets, I just don't know
 * how to start them yet).
 */
void Implementations::viewLinkGraph()
{
	if ( dot_graph_ == "" ) {
		ROS_WARN_STREAM("QOrchestra: no implementation loaded yet, aborting view.");
		return;
	}
	// we write every time only here since qtemporaryfile is a bit narky about threads.
	if ( dot_graph_file.open() ) {
		ROS_INFO_STREAM("QOrchestra: writing dot file [" << dot_graph_file.fileName().toStdString() << "]");
		QTextStream out(&dot_graph_file);
		out << QString(dot_graph_.c_str());
		dot_graph_file.close();
	}

	QStringList args;
	QProcess *dot_graph_viewer = new QProcess();
	args << dot_graph_file.fileName();
	dot_graph_viewer->start("kgraphviewer",args);
	external_processes.push_back(dot_graph_viewer);
}

} // namespace rocon_qorchestra

