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
#include <ctime>

#include <iostream>
#include <tuple>
#include <functional>
#include <exception>
#include <pcl/octree/octree.h>
#include <pcl/octree/impl/octree_base.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>
#include "OctreePointCloudDynamic.h"

typedef OctreePointCloudDynamic<pcl::PointXYZL> OctreeT;

void extractOctree(ErlNifEnv* env, ERL_NIF_TERM term, OctreeT* octree)
		throw (nifpp::badarg) {
	nifpp::str_atom atom;
	unsigned long int reference;
	auto octree_in = std::make_tuple(ref(atom), ref(reference), ref(octree));
	nifpp::get_throws(env, term, octree_in);
	if (atom.compare("octree_nif_t") != 0) {
		throw nifpp::badarg();
	}
}

void extractPoint(ErlNifEnv* env, ERL_NIF_TERM term, OctreeT::PointT* point)
		throw (nifpp::badarg) {
	double x, y, z;
	auto tup_in = std::make_tuple(ref(x), ref(y), ref(z));
	nifpp::get_throws(env, term, tup_in);
	point->x = x;
	point->y = y;
	point->z = z;
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
		return nifpp::make(env,
				std::make_tuple(nifpp::str_atom("ok"),
						std::make_tuple(nifpp::str_atom("octree_nif_t"),
								enif_make_ref(env), octree_ptr)));
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		return enif_make_badarg(env);
	}

}

//static ERL_NIF_TERM destroyO(ErlNifEnv* env, int argc,
//		const ERL_NIF_TERM argv[]) {
//	if (argc != 1)
//		return enif_make_badarg(env);
//	try {
//		OctreeT* octree = nullptr;
//		extractOctree(env, argv[0], octree);
//		nifpp::resource_ptr<OctreeT> octree_ptr;
//		nifpp::get(env, argv[0], octree_ptr);
//		std::cout << octree << std::endl;
//		return nifpp::make(env, nifpp::str_atom("ok"));
//	} catch (std::exception& e) {
//		std::cout << e.what() << std::endl;
//		return enif_make_badarg(env);
//	}
//}

static ERL_NIF_TERM insertE(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 2)
		return enif_make_badarg(env);
	try {
		OctreeT* octree = nullptr;
		extractOctree(env, argv[0], octree);
		OctreeT::PointT point;
		extractPoint(env, argv[1], &point);

		OctreeT::HandleT handle = octree->addPoint(point);

		return nifpp::make(env,
				std::make_tuple(nifpp::str_atom("ok"),
						nifpp::make(env, handle)));
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		return enif_make_badarg(env);
	}
}

static ERL_NIF_TERM updateE(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 3)
		return enif_make_badarg(env);
	try {
		OctreeT* octree = nullptr;
		OctreeT::HandleT handle;
		OctreeT::PointT point;

		extractOctree(env, argv[0], octree);
		nifpp::get_throws(env, argv[1], handle);
		extractPoint(env, argv[2], &point);

		octree->updatePoint(handle, point);

		return nifpp::make(env, nifpp::str_atom("ok"));
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		return enif_make_badarg(env);
	}
}

static ERL_NIF_TERM removeE(ErlNifEnv* env, int argc,
		const ERL_NIF_TERM argv[]) {
	if (argc != 2)
		return enif_make_badarg(env);
	try {
		OctreeT* octree = nullptr;
		OctreeT::HandleT handle;

		extractOctree(env, argv[0], octree);
		nifpp::get_throws(env, argv[1], handle);

		octree->removePoint(handle);

		return nifpp::make(env, nifpp::str_atom("ok"));
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		return enif_make_badarg(env);
	}
}

static ErlNifFunc nif_funcs[] = {
//
		{ "create", 0, createO },
		//
		// { "destroy", 1, destroyO },
		//
		{ "insert", 2, insertE },
		//
		{ "update", 3, updateE },
		//
		{ "remove", 2, removeE } };

ERL_NIF_INIT(octree_nif, nif_funcs, load, NULL, NULL, NULL)

}
