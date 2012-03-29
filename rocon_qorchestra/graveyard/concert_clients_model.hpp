/**
 * @file /rocon_qorchestra/include/rocon_qorchestra/tree_model.hpp
 *
 * @brief Item for a simple tree model
 *
 * Code comes from the simple tree model example in qt assistant.
 *
 * @date 20/11/2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef QORCHESTRA_CONCERT_CLIENTS_MODEL_HPP_
#define QORCHESTRA_CONCERT_CLIENTS_MODEL_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <concert_comms/ConcertClients.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_qorchestra {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class TreeItem;

class ConcertClientsModel : public QAbstractItemModel {
	Q_OBJECT
public:
	ConcertClientsModel(QObject *parent = 0);
	~ConcertClientsModel();

	QVariant data(const QModelIndex &index, int role) const;
	Qt::ItemFlags flags(const QModelIndex &index) const;
	QVariant headerData(int section, Qt::Orientation orientation,
				 int role = Qt::DisplayRole) const;
	QModelIndex index(int row, int column,
			   const QModelIndex &parent = QModelIndex()) const;
	QModelIndex parent(const QModelIndex &index) const;
	int rowCount(const QModelIndex &parent = QModelIndex()) const;
	int columnCount(const QModelIndex &parent = QModelIndex()) const;
	void clear();

private:
	void reloadData(const concert_comms::ConcertClientsConstPtr concert);
	void setupModelData(const QStringList &lines, TreeItem *parent);
	TreeItem *rootItem;
};

} // namespace rocon_qorchestra

#endif /* QORCHESTRA_CONCERT_CLIENTS_MODEL_HPP_ */
