/**
 * @file /rocon_qorchestra/include/rocon_qorchestra/implementation.hpp
 *
 * Stores implementation details in a qt model with a convenient c++
 * interface.
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ROCON_QORCHESTRA_IMPLEMENTATION_HPP_
#define ROCON_QORCHESTRA_IMPLEMENTATION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMutex>
#include <QObject>
#include <QProcess>
#include <QStandardItemModel>
#include <QTemporaryFile>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_qorchestra {

/*****************************************************************************
** Interface
*****************************************************************************/

class Implementations : public QObject {
	Q_OBJECT
public:
	virtual ~Implementations();
	bool fetch();
	void viewLinkGraph();
	QStandardItemModel* model() { return &model_; }

Q_SIGNALS:
	void sigImplementationsFetched();

private:
    QStandardItemModel model_;
    QMutex mutex_;
    std::string dot_graph_;
    QTemporaryFile dot_graph_file;
    std::vector<QProcess*> external_processes;
};


} // namespace rocon_qorchestra


#endif /* ROCON_QORCHESTRA_IMPLEMENTATION_HPP_ */
