/**
 * @file /rocon_qorchestra/src/tree_model.cpp
 *
 * @brief Item for a simple tree model
 *
 * Code comes from the simple tree model example in qt assistant.
 *
 * @date 20/11/2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include "../include/rocon_qorchestra/tree_item.hpp"
#include "../include/rocon_qorchestra/concert_clients_model.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_qorchestra {

/*****************************************************************************
** Implementation
*****************************************************************************/

ConcertClientsModel::ConcertClientsModel(QObject *parent) : QAbstractItemModel(parent)
{
	QList<QVariant> rootData;
	rootData << "Concert Client" << "Description";
	QString data =
    QString("Getting Started                         \tHow to familiarize yourself with Qt Designer\n") +
    QString("    Launching Designer                  \tRunning the Qt Designer application\n") +
    QString("    The User Interface                  \tHow to interact with Qt Designer\n") +
    QString("Connection Editing Mode                 \tConnecting widgets together with signals and slots\n") +
    QString("    Connecting Objects                  \tMaking connections in Qt Designer\n") +
    QString("    Editing Connections                 \tChanging existing connections\n");

	rootItem = new TreeItem(rootData);
	setupModelData(data.split(QString("\n")), rootItem);
}


ConcertClientsModel::~ConcertClientsModel()
{
	delete rootItem;
}

int ConcertClientsModel::columnCount(const QModelIndex &parent) const
{
  if (parent.isValid())
    return static_cast<TreeItem*>(parent.internalPointer())->columnCount();
  else
    return rootItem->columnCount();
}

QVariant ConcertClientsModel::data(const QModelIndex &index, int role) const
{
  if (!index.isValid())
    return QVariant();
  if (role != Qt::DisplayRole)
    return QVariant();
  TreeItem *item = static_cast<TreeItem*>(index.internalPointer());
  return item->data(index.column());
}

Qt::ItemFlags ConcertClientsModel::flags(const QModelIndex &index) const
{
  if (!index.isValid())
    return 0;
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

QVariant ConcertClientsModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    return rootItem->data(section);
  return QVariant();
}

QModelIndex ConcertClientsModel::index(int row, int column, const QModelIndex &parent) const
{
  if (!hasIndex(row, column, parent))
    return QModelIndex();

  TreeItem *parentItem;
  if (!parent.isValid())
    parentItem = rootItem;
  else
    parentItem = static_cast<TreeItem*>(parent.internalPointer());

  TreeItem *childItem = parentItem->child(row);
  if (childItem)
    return createIndex(row, column, childItem);
  else
    return QModelIndex();
}

QModelIndex ConcertClientsModel::parent(const QModelIndex &index) const
{
  if (!index.isValid())
    return QModelIndex();

  TreeItem *childItem = static_cast<TreeItem*>(index.internalPointer());
  TreeItem *parentItem = childItem->parent();
  if (parentItem == rootItem)
    return QModelIndex();

  return createIndex(parentItem->row(), 0, parentItem);
}

int ConcertClientsModel::rowCount(const QModelIndex &parent) const {
    TreeItem *parentItem;
    if (parent.column() > 0)
        return 0;

    if (!parent.isValid())
        parentItem = rootItem;
    else
        parentItem = static_cast<TreeItem*>(parent.internalPointer());

    return parentItem->childCount();
}

void ConcertClientsModel::clear() {

}

void ConcertClientsModel::reloadData(const concert_comms::ConcertClientsConstPtr concert) {
	for ( unsigned int i = 0; i < concert->clients.size(); ++i ) {
		ROS_DEBUG_STREAM("Concert Clients: =========== Concert Client List ===========");
		ROS_DEBUG_STREAM("Concert Clients: \t unique name: " << concert->clients[i].unique_name );
		ROS_DEBUG_STREAM("Concert Clients: \t platform: " << concert->clients[i].platform );
		ROS_DEBUG_STREAM("Concert Clients: \t system: " << concert->clients[i].system );
		ROS_DEBUG_STREAM("Concert Clients: \t suggested_name: " << concert->clients[i].suggested_name );
		ROS_DEBUG_STREAM("Concert Clients: \t key: " << concert->clients[i].key );
		ROS_DEBUG_STREAM("Concert Clients: \t app_manager_uri: " << concert->clients[i].app_manager_uri );
		ROS_DEBUG_STREAM("Concert Clients: \t last_connection_timestamp: " << concert->clients[i].last_connection_timestamp );
		if ( concert->clients[i].is_connected ) {
			ROS_DEBUG_STREAM("Concert Clients: \t is_connected : yes" );
		} else{
			ROS_DEBUG_STREAM("Concert Clients: \t is_connected : no" );
		}
		ROS_DEBUG_STREAM("Concert Clients: \t zeroconf info: todo" );
	}
}

/**
 * This is pulling information from a file with 'lines' of text (QStringList).
 * See the constructor for more it's caller.
 *
 * @param lines
 * @param parent
 */
void ConcertClientsModel::setupModelData(const QStringList &lines, TreeItem *parent) {
    QList<TreeItem*> parents;
    QList<int> indentations;
    parents << parent;
    indentations << 0;

    int number = 0;

    while (number < lines.count()) {
        int position = 0;
        while (position < lines[number].length()) {
            if (lines[number].mid(position, 1) != " ")
                break;
            position++;
        }
        QString lineData = lines[number].mid(position).trimmed();

        if (!lineData.isEmpty()) {
            // Read the column data from the rest of the line.
            QStringList columnStrings = lineData.split("\t", QString::SkipEmptyParts);
            QList<QVariant> columnData;
            for (int column = 0; column < columnStrings.count(); ++column)
                columnData << columnStrings[column];

            if (position > indentations.last()) {
                // The last child of the current parent is now the new parent
                // unless the current parent has no children.

                if (parents.last()->childCount() > 0) {
                    parents << parents.last()->child(parents.last()->childCount()-1);
                    indentations << position;
                }
            } else {
                while (position < indentations.last() && parents.count() > 0) {
                    parents.pop_back();
                    indentations.pop_back();
                }
            }
            // Append a new item to the current parent's list of children.
            parents.last()->appendChild(new TreeItem(columnData, parents.last()));
        }

        number++;
    }
}

} // namespace rocon_qorchestra
