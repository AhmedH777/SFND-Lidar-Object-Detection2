/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertRecursion(Node<PointT>** node, int depth, PointT point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node<PointT>(point,id);
		}
		else
		{
			int cd = depth % 2;
			float ptNode,pt;

			if(cd == 0)
			{
				pt = point.x;
				ptNode = ((*node)->point.x);
			}
			else
			{
				pt = point.y;
				ptNode = ((*node)->point.y);
			}


			if(pt < ptNode)
			{
				insertRecursion(&((*node)->left),depth +1,point,id);
			}
			else
			{
				insertRecursion(&((*node)->right),depth +1,point,id);
			}
		}
	}
	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertRecursion(&root,0,point,id);

	}


	void searchRecursion(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if( ( node->point.x >= (target.x - distanceTol) ) &&
				( node->point.x <= (target.x + distanceTol) ) &&
				( node->point.y >= (target.y - distanceTol) ) &&
				( node->point.y <= (target.y + distanceTol) ) )
				{
					float dxSq = ( (node->point.x - target.x) * (node->point.x - target.x) );
					float dySq = ( (node->point.y - target.y) * (node->point.y - target.y) );
					float distance = sqrt(dxSq +dySq);

					if(distance < distanceTol)
					{
						ids.push_back(node->id);
					}
				}

			int cd = depth % 2;
			float targetPt,pt;
			if(cd == 0)
			{
				pt = node->point.x;
				targetPt = target.x;
			}
			else
			{
				pt = node->point.y;
				targetPt = target.y;
			}

			if( ( targetPt - distanceTol ) < pt )
			{
				searchRecursion(target, node->left, ( depth + 1 ), distanceTol, ids);
			}
			if( ( targetPt + distanceTol ) > pt )
			{
				searchRecursion(target, node->right, ( depth + 1 ), distanceTol, ids);
			}

		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;

		searchRecursion(target, this->root, 0, distanceTol, ids);
		return ids;
	}
	

};




