/*
 * OctreePointCloudDynamic.h
 *
 *  Created on: 03.03.2014
 *      Author: lemo
 */

#ifndef OCTREEPOINTCLOUDDYNAMIC_H_
#define OCTREEPOINTCLOUDDYNAMIC_H_

#include <queue>
#include <exception>
#include <pcl/octree/octree.h>

template<typename PointTArg, typename BaseClass = pcl::octree::OctreePointCloud<
		PointTArg> >
class OctreePointCloudDynamic: public BaseClass {
public:
	typedef PointTArg PointT;
	typedef int HandleT;
	typedef std::queue<HandleT> QueueT;

	QueueT queue_;
	typename BaseClass::PointCloudPtr pointcloud_;
	typename BaseClass::IndicesPtr indices_;
	typename pcl::PointCloud<PointT>::Ptr input__;

	OctreePointCloudDynamic(const double resolution) :
			BaseClass(resolution), queue_(), pointcloud_(
					new typename BaseClass::PointCloud()), indices_(
					new std::vector<int>()) {
		setInputCloud(pointcloud_, indices_);

	}
	virtual ~OctreePointCloudDynamic() {
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
		pcl::octree::OctreeContainerPointIndices* oldcontainer =
				this->findLeafAtPoint(oldpoint);
		pcl::octree::OctreeContainerPointIndices* container =
				this->findLeafAtPoint(point);

		if (container == oldcontainer) {
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
