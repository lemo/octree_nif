%% @author lemo
%% @doc @todo Add description to octree_nif.


-module(octree_nif).

-include("octree_nif.hrl").

%% ====================================================================
%% API functions
%% ====================================================================
-export([create/0, insert/2, update/3, remove/2, init/0,nearestSearch/3, radiusSearch/4]).


-on_load(init/0).


init() ->
    Nif = case code:priv_dir(?MODULE) of
                 {error, bad_name} ->
                     case filelib:is_dir(filename:join(["..", "priv"])) of
                         true ->
                             filename:join(["..", "priv", "octree_nif"]);
                         false ->
                             filename:join(["priv", "octree_nif"])
                     end;
                 Dir ->
                     filename:join(Dir, "octree_nif")
             end,
	io:format("loading ~p ",[Nif]),
    Ret = (catch erlang:load_nif(Nif, 0)),
	io:format("~p~n",[Ret]),
    case erlang:system_info(otp_release) of
        "R13B03" -> true;
        _ -> ok
    end.

create() ->
    exit(octree_nif_not_loaded).
insert(_Octree, _Point) ->
    exit(octree_nif_not_loaded).
update(_Octree, _Index, _Point) ->
    exit(octree_nif_not_loaded).
remove(_Octree, _Index) ->
    exit(octree_nif_not_loaded).

radiusSearch(_Octree, _Index, _Radius, _MaxAmount) ->
	exit(octree_nif_not_loaded).
nearestSearch(_Octree, _Index, _MaxAmount) ->
	exit(octree_nif_not_loaded).



