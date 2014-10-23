//============================================================================
// Name        : octree_nif.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <cstddef>
#include <cstdlib>

#include <erl_nif.h>

void* operator new(size_t sz) {
	return enif_alloc(sz);
}

void operator delete(void* m) noexcept (true) {
	enif_free(m);
}

#include <nifpp.h>

#include <iostream>
#include <vector>
#include <list>
#include <ctime>
#include <tuple>
#include <functional>
#include <exception>

#include <pcl/octree/octree.h>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include <pcl/octree/impl/octree_search.hpp>
#include "OctreePointCloudDynamic.h"

typedef pcl::PointXYZL PointT;
typedef OctreePointCloudDynamic<PointT,
		pcl::octree::OctreePointCloudSearch<PointT>> OctreeT;

OctreeT* extractOctree(ErlNifEnv* env, ERL_NIF_TERM term) throw (nifpp::badarg) {
	nifpp::str_atom atom;
	uintptr_t reference;
	OctreeT* octree;
	auto octree_in = std::make_tuple(ref(atom), ref(reference), ref(octree));
	nifpp::get_throws(env, term, octree_in);
	if (atom.compare("octree_nif_t") != 0) {
		throw nifpp::badarg();
	}
	return octree;
}

void extractPoint(ErlNifEnv* env, ERL_NIF_TERM term, OctreeT* octree,
		OctreeT::PointT* point) throw (nifpp::badarg) {

	if (enif_is_number(env, term)) {
		OctreeT::HandleT handle;
		nifpp::get(env, term, handle);
		point->x = octree->input__->at(handle).x;
		point->y = octree->input__->at(handle).y;
		point->z = octree->input__->at(handle).z;
		point->label = handle;
	} else {
		double x, y, z;
		int l;
		nifpp::str_atom atom;
		auto tup_in = std::make_tuple(ref(atom), ref(x), ref(y), ref(z),
				ref(l));
		nifpp::get_throws(env, term, tup_in);
		if (atom.compare("octree_point") != 0) {
			throw nifpp::badarg();
		}
		point->x = x;
		point->y = y;
		point->z = z;
		point->label = l;
	}
}

nifpp::TERM genResultList(ErlNifEnv* env, std::vector<int> indices,
		std::vector<float> squaredDistances) {
	std::list<nifpp::TERM> list;
	for (size_t i = 0; i < indices.size(); ++i) {
		list.push_back(
				nifpp::make(env,
						std::make_tuple(nifpp::make(env, indices[i]),
								nifpp::make(env, squaredDistances[i]))));
	}
	return nifpp::make(env, list);

}

extern "C" {

static int load(ErlNifEnv* env, void** priv, ERL_NIF_TERM load_info) {
	nifpp::register_resource<OctreeT>(env, nullptr, "octree");
	return 0;
}

static ERL_NIF_TERM createO(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 0)
		return enif_make_badarg(env);

	try {
		double resolution = 1000;

		nifpp::resource_ptr<OctreeT> octree_ptr = nifpp::construct_resource<
				OctreeT>(resolution);

		uintptr_t reference = reinterpret_cast<uintptr_t>(octree_ptr.get());
		return nifpp::make(env,
				std::make_tuple(nifpp::str_atom("ok"),
						std::make_tuple(nifpp::str_atom("octree_nif_t"),
								nifpp::make(env, reference), octree_ptr)));
	} catch (...) {
		return enif_make_badarg(env);
	}

}

static ERL_NIF_TERM insertE(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 2)
		return enif_make_badarg(env);

	try {
		OctreeT* octree = extractOctree(env, argv[0]);
		OctreeT::PointT point;
		extractPoint(env, argv[1], octree, &point);

		OctreeT::HandleT handle = octree->addPoint(point);

		return nifpp::make(env,
				std::make_tuple(nifpp::str_atom("ok"),
						nifpp::make(env, handle)));
	} catch (...) {
		return enif_make_badarg(env);
	}
}

static ERL_NIF_TERM updateE(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 3)
		return enif_make_badarg(env);
	try {
		OctreeT::HandleT handle;
		OctreeT::PointT point;
		OctreeT* octree = extractOctree(env, argv[0]);
		nifpp::get_throws(env, argv[1], handle);
		extractPoint(env, argv[2], octree, &point);

		octree->updatePoint(handle, point);

		return nifpp::make(env, nifpp::str_atom("ok"));
	} catch (...) {
		return enif_make_badarg(env);
	}

}

static ERL_NIF_TERM removeE(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 2)
		return enif_make_badarg(env);
	try {
		OctreeT::HandleT handle;

		OctreeT* octree = extractOctree(env, argv[0]);
		nifpp::get_throws(env, argv[1], handle);

		octree->removePoint(handle);

		return nifpp::make(env, nifpp::str_atom("ok"));
	} catch (...) {
		return enif_make_badarg(env);
	}
}

static ERL_NIF_TERM radiusSearch(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 4)
		return enif_make_badarg(env);
	try {
		OctreeT::PointT p_q;
		//OctreeT::HandleT handle;
		double radius;
		unsigned int max_nn = 0;

		OctreeT* octree = extractOctree(env, argv[0]);
		extractPoint(env, argv[1], octree, &p_q);
		nifpp::get_throws(env, argv[2], radius);
		nifpp::get_throws(env, argv[3], max_nn);

		std::vector<int> k_indices;
		std::vector<float> k_sqr_distances;

		int found_amount = octree->radiusSearch(p_q, radius, k_indices,
				k_sqr_distances, max_nn);

		nifpp::TERM list = genResultList(env, k_indices, k_sqr_distances);

		return nifpp::make(env, std::make_tuple(nifpp::str_atom("ok"), list));
	} catch (...) {
		return enif_make_badarg(env);
	}
}

static ERL_NIF_TERM nearestSearch(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 3)
		return enif_make_badarg(env);
	try {
		//OctreeT::HandleT handle;
		OctreeT::PointT p_q;
		int amount;

		OctreeT* octree = extractOctree(env, argv[0]);
		//nifpp::get_throws(env, argv[1], handle);
		extractPoint(env, argv[1], octree, &p_q);
		nifpp::get_throws(env, argv[2], amount);

		std::vector<int> k_indices;
		std::vector<float> k_sqr_distances;

		int found_amount = octree->nearestKSearch(p_q, amount, k_indices,
				k_sqr_distances);

		nifpp::TERM list = genResultList(env, k_indices, k_sqr_distances);

		return nifpp::make(env, std::make_tuple(nifpp::str_atom("ok"), list));
	} catch (...) {
		return enif_make_badarg(env);
	}
}

static ErlNifFunc nif_funcs[] = {

//
		{ "create", 0, createO },
		//
		{ "insert", 2, insertE },
		//
		{ "update", 3, updateE },
		//
		{ "remove", 2, removeE },
		//
		{ "radiusSearch", 4, radiusSearch },
		//
		{ "nearestSearch", 3, nearestSearch } };

ERL_NIF_INIT(octree_nif, nif_funcs, load, NULL, NULL, NULL)

}
