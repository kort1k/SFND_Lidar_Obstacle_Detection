/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(const std::vector<float> &point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

        // @kort1k
		insertTraverse(point, id, &root, 0);
	}

	void insertTraverse(const std::vector<float> &point, int id, Node** node, uint depth)
	{
        // @kort1k
		if (*node) {
			uint d = depth % point.size();
			if ((*node)->point[d] > point[d]) {
				insertTraverse(point, id, &((*node)->left), depth + 1);			
			}
			else {
				insertTraverse(point, id, &((*node)->right), depth + 1);
			}
		}
		else{
			*node = new Node(point, id);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchTraverse(target, distanceTol, root, ids, 0);
		return ids;
	}
	
	void searchTraverse(const std::vector<float> &target, float tol, Node* node, std::vector<int>& ids, uint depth)
	{
        // @kort1k
		if (node) {
			float p0 = node->point[0], t0 = target[0];
			float p1 = node->point[1], t1 = target[1];
			if (   p0 >= (t0-tol) && p0 <= (t0+tol)
				&& p1 >= (t1-tol) && p1 <= (t1+tol)) {
					float a = p0-t0;
					float b = p1-t1;
					float d = sqrt(a*a + b*b);
					if (d <= tol)
						ids.push_back(node->id);
			}

			uint d = depth % target.size();
			if ( (target[d] - tol) < node->point[d]) {
				searchTraverse(target, tol, node->left, ids, depth + 1);			
			}
			if ((target[d] + tol) > node->point[d])  {
				searchTraverse(target, tol, node->right, ids, depth + 1);
			}
		}
		else
			return;
	}

};




