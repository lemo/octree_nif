/*
 * OctreePointCloudDynamic.h
 *
 *  Created on: 03.03.2014
 *      Author: lemo
 */

#ifndef OCTREEPOINTCLOUDDYNAMIC_H_
#define OCTREEPOINTCLOUDDYNAMIC_H_

#include <queue>
#include <pcl/octree/octree.h>
#define DEBUG
#ifdef DEBUG
#include <iostream>
#endif

template<typename PointTArg, typename BaseClass = pcl::octree::OctreePointCloud<
		PointTArg> >
class OctreePointCloudDynamic: BaseClass {
public:
	typedef PointTArg PointT;
	typedef int HandleT;

	std::queue<HandleT> queue_;
	typename pcl::PointCloud<PointT>::Ptr input__;

	OctreePointCloudDynamic(const double resolution) :
			BaseClass(resolution), queue_() {
#ifdef DEBUG
		std::cout << "+";
#endif
	}
	virtual ~OctreePointCloudDynamic() {
#ifdef DEBUG
		std::cout << "-";
#endif
	}

	HandleT addPoint(PointT point) {
		int index = 0;
		if (queue_.empty()) {
			index = this->input_->points.size();
			(*this->input__).push_back(point);
		} else {
			index = queue_.front();
			queue_.pop();
			(*this->input__)[index] = point;
		}
		this->addPointIdx(static_cast<const int>(index));
		return index;
	}

	void updatePoint(HandleT handle, PointT point) {
		PointT& oldpoint = (*this->input__)[handle];
		pcl::octree::OctreeContainerPointIndices* container =
				this->findLeafAtPoint(oldpoint);
		oldpoint = point;
		if (container != this->findLeafAtPoint(point)) {
			if (container->getSize() <= 1) {
				this->removeLeaf(point.y, point.y, point.z);
			} else {
				std::vector<int>& vec = container->getPointIndicesVector();
				for (std::vector<int>::iterator it = vec.begin();
						it != vec.end(); ++it) {
					if (handle == (*it)) {
						vec.erase(it);
						break;
					}
				}
			}
			this->addPointIdx(handle);
		}
	}

	void removePoint(HandleT handle) {
		const PointT& point = (*this->input_)[handle];
		pcl::octree::OctreeKey key;
		this->genOctreeKeyforPoint(point, key);
		pcl::octree::OctreeContainerPointIndices* container = this->findLeaf(
				key);
		if (container->getSize() <= 1) {
			this->removeLeaf(key);
		} else {
			std::vector<int>& vec = container->getPointIndicesVector();
			for (std::vector<int>::iterator it = vec.begin(); it != vec.end();
					++it) {
				if (handle == (*it)) {
					vec.erase(it);
					break;
				}
			}
		}
		queue_.push(handle);
	}

	void setInputCloud(const typename BaseClass::PointCloudPtr &cloud_arg,
			const typename BaseClass::IndicesConstPtr &indices_arg =
					typename BaseClass::IndicesConstPtr()) {
		BaseClass::setInputCloud(cloud_arg, indices_arg);
		input__ = cloud_arg;
	}

};

#endif /* OCTREEPOINTCLOUDDYNAMIC_H_ */
