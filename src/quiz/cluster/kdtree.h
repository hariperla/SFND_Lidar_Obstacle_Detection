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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	// Helper function to call inside insert function to insert into the kdtree
	void insertHelp(Node* &node, unsigned depth, std::vector<float> point, int id)
	{
		// Point a new node to root if it's a null pointer
		if (node == nullptr)
		{
			node = new Node(point,id);
			return;
		}
		else
		{
			unsigned rem = depth % 2; // To traverse through the left and right of the tree of a 2d tree
			
			// If curr point less than the point in the node insert left else insert right
			if (point[rem] < node->point[rem])
			{
				insertHelp(node->left, depth+1, point, id);
			}
			else
			{
				insertHelp(node->right, depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelp(root, 0, point, id);
		
	}

	// Search helper to be used to search inside a kd tree
	void searchHelp(Node* node,std::vector<float> target, unsigned depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != nullptr)
		{
			// Check if we are with in a bounding box first on both x and y direction
			if ( node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) && 
				node->point[1] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) )
				{
					// Calculate distance
					float dist = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + 
									  (node->point[1] - target[1]) * (node->point[1] - target[1]));
					
					if (dist <= distanceTol)
					{
						ids.push_back(node->id);
					}
				}
			
			// Check box boundary and traverse left or right
			if ((target[depth%2] - distanceTol) < node->point[depth%2])
			{
				searchHelp(node->left, target, depth+1, distanceTol, ids);
			}
			if ((target[depth%2] + distanceTol) > node->point[depth%2])
			{
				searchHelp(node->right, target, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelp(root, target, 0, distanceTol, ids);
		return ids;
	}
	

};




