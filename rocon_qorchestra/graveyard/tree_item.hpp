/**
 * @file /rocon_qorchestra/include/rocon_qorchestra/tree_item.hpp
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

#ifndef QORCHESTRA_TREE_ITEM_HPP_
#define QORCHESTRA_TREE_ITEM_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QList>
#include <QVariant>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_qorchestra {

/*****************************************************************************
** Interface
*****************************************************************************/

class TreeItem {
public:
	TreeItem(const QList<QVariant> &data, TreeItem *parent = 0);
	~TreeItem();

	void appendChild(TreeItem *child);

	TreeItem *child(int row);
	int childCount() const;
	int columnCount() const;
	QVariant data(int column) const;
	int row() const;
	TreeItem *parent();
	void clear();

private:
	QList<TreeItem*> childItems;
	QList<QVariant> itemData;
	TreeItem *parentItem;
};

} // namespace rocon_qorchestra

#endif /* QORCHESTRA_TREE_ITEM_HPP_ */
