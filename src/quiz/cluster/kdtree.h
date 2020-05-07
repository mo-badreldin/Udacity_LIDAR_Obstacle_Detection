/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>




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

	void insertHelper(Node* &curr_node_ptr,Node* &new_node_ptr,int axis)
	{
		//0 for x axis, 1 for y axis, 2 for z axis
		int new_axis = (axis == 2) ? 0 : axis+1;

		if(curr_node_ptr == NULL)
		{
			curr_node_ptr = new_node_ptr;
		}
		else
		{
			if(new_node_ptr->point[axis] < curr_node_ptr->point[axis])
			{
				insertHelper(curr_node_ptr->left,new_node_ptr,new_axis);
			}
			else
			{
				insertHelper(curr_node_ptr->right,new_node_ptr,new_axis);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		//0 for x axis / 1 for y axis
		Node* new_node (new Node(point,id));
		insertHelper(root,new_node,0);
	}

	void searchHelper(Node* &cmp_node_ptr,int axis,std::vector<float> &target,float distanceTol,std::vector<int> &ids)
	{
		int new_axis = (axis == 2) ? 0 : axis+1;


		if(cmp_node_ptr == NULL)
		{
			return;
		}

		if(cmp_node_ptr->point[0] >= (target[0] - distanceTol) && cmp_node_ptr->point[0] <= (target[0] + distanceTol)
				&& cmp_node_ptr->point[1] >= (target[1] - distanceTol) && cmp_node_ptr->point[1] <= (target[1] + distanceTol))
		{
			// current node point lies within the box of target point
			float distance = sqrt ( pow(target[0]- cmp_node_ptr->point[0],2) + pow(target[1]-cmp_node_ptr->point[1],2) );

			if(distance <= distanceTol)
			{
				ids.push_back(cmp_node_ptr->id);
				searchHelper(cmp_node_ptr->left,new_axis,target,distanceTol,ids);
				searchHelper(cmp_node_ptr->right,new_axis,target,distanceTol,ids);

			}
		}
		else if( (target[axis] + distanceTol) < cmp_node_ptr->point[axis])
		{
			// target point, lies to the left of the current node point exclude right part of the search tree
			searchHelper(cmp_node_ptr->left,new_axis,target,distanceTol,ids);

		}
		else if( (target[axis] - distanceTol) > cmp_node_ptr->point[axis])
		{
			// target point, lies to the right of the current node point exclude left part of the search tree
			searchHelper(cmp_node_ptr->right,new_axis,target,distanceTol,ids);
		}
		else
		{
			// target point box, lies in both left and right part of tree, continue search in both branches
			searchHelper(cmp_node_ptr->left,new_axis,target,distanceTol,ids);
			searchHelper(cmp_node_ptr->right,new_axis,target,distanceTol,ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		auto startTime = std::chrono::steady_clock::now();

		searchHelper(root,0,target,distanceTol,ids);

		auto endTime = std::chrono::steady_clock::now();
		auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
		//std::cout << "Searching Took: " <<  elapsedTime.count() << " microseconds" << std::endl;
		return ids;
	}

};






