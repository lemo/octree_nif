%% @author lemo
%% @doc @todo Add description to octree.


-module(octree).
-behaviour(gen_server).
-export([init/1, handle_call/3, handle_cast/2, handle_info/2, terminate/2, code_change/3]).

-include("octree_nif.hrl").

%% ====================================================================
%% API functions
%% ====================================================================
-export([start_link/0,
		 insert/2,
		 update/3,
		 remove/2,
 		 radiusSearch/4,
		 nearestSearch/3]).


start_link() ->
	gen_server:start_link(?MODULE, [], []).

% return a handle
insert(Octree, Point) ->
    gen_server:call(Octree, {insert, Point}).

update(Octree, Index, Point) ->
    gen_server:call(Octree, {update, Index, Point}).

remove(Octree, Index) ->
    gen_server:call(Octree, {remove, Index}).

radiusSearch(Octree, IndexOrVec, Radius, MaxAmount) ->
	gen_server:call(Octree, {radiusSearch, IndexOrVec, Radius, MaxAmount}).

nearestSearch(Octree, IndexOrVec, MaxAmount) ->
	gen_server:call(Octree, {nearestSearch, IndexOrVec, MaxAmount}).


%% ====================================================================
%% Behavioural functions 
%% ====================================================================
-record(state, {octree, pid_array}).

%% init/1
%% ====================================================================
%% @doc <a href="http://www.erlang.org/doc/man/gen_server.html#Module:init-1">gen_server:init/1</a>
-spec init(Args :: term()) -> Result when
	Result :: {ok, State}
			| {ok, State, Timeout}
			| {ok, State, hibernate}
			| {stop, Reason :: term()}
			| ignore,
	State :: term(),
	Timeout :: non_neg_integer() | infinity.
%% ====================================================================
init([]) ->
	{ok, Octree} = octree_nif:create(),
    {ok, #state{octree = Octree, pid_array = array:new()}}.


%% handle_call/3
%% ====================================================================
%% @doc <a href="http://www.erlang.org/doc/man/gen_server.html#Module:handle_call-3">gen_server:handle_call/3</a>
-spec handle_call(Request :: term(), From :: {pid(), Tag :: term()}, State :: term()) -> Result when
	Result :: {reply, Reply, NewState}
			| {reply, Reply, NewState, Timeout}
			| {reply, Reply, NewState, hibernate}
			| {noreply, NewState}
			| {noreply, NewState, Timeout}
			| {noreply, NewState, hibernate}
			| {stop, Reason, Reply, NewState}
			| {stop, Reason, NewState},
	Reply :: term(),
	NewState :: term(),
	Timeout :: non_neg_integer() | infinity,
	Reason :: term().
%% ====================================================================
handle_call({insert, Point}, From, State) ->
	{ok, Handle} = octree_nif:insert(State#state.octree, Point),
	NArray = array:set(Handle, From, State#state.pid_array),
	{reply, {ok, Handle} , State#state{pid_array = NArray}};

handle_call({update, Index, Point}, _From, State) ->
	%io:format("UPDATE~n~p ~p ~p ~n~p ~n",[State#state.octree, Index, Point, State]),
	ok = octree_nif:update(State#state.octree, Index, Point),
	{reply, ok, State#state{}}; 

handle_call({remove, Index}, _From, State) ->
	ok = octree_nif:remove(State#state.octree, Index),
	NArray = array:reset(Index, State#state.pid_array),
	{reply, ok, State#state{pid_array = NArray}};

handle_call({radiusSearch, Index, Radius, MaxAmount}, _From, State) when is_integer(Index)->
	{ok, List} = octree_nif:radiusSearch(State#state.octree, Index, Radius, MaxAmount),
	RetList = [array:get(Handle, State#state.pid_array) || {Handle, _Distance} <- List],
	{reply, {ok, RetList}, State};

handle_call({radiusSearch, Point, Radius, MaxAmount}, _From, State) when is_record(Point, octree_point) ->
	{ok, List} = octree_nif:radiusSearch(State#state.octree, Point, Radius, MaxAmount),
	RetList = [array:get(Handle, State#state.pid_array) || {Handle, _Distance} <- List],
	{reply, {ok, RetList}, State};

handle_call({nearestSearch, Index, MaxAmount}, _From, State) when is_integer(Index) ->
	{ok, List} = octree_nif:nearestSearch(State#state.octree, Index, MaxAmount),
	RetList = [array:get(Handle, State#state.pid_array) || {Handle, _Distance} <- List],
    {reply, {ok, RetList}, State};

handle_call({nearestSearch, Point, MaxAmount}, _From, State) when is_record(Point, octree_point) ->
	{ok, List} = octree_nif:nearestSearch(State#state.octree, Point, MaxAmount),
	RetList = [array:get(Handle, State#state.pid_array) || {Handle, _Distance} <- List],
    {reply, {ok, RetList}, State}.


%% handle_cast/2
%% ====================================================================
%% @doc <a href="http://www.erlang.org/doc/man/gen_server.html#Module:handle_cast-2">gen_server:handle_cast/2</a>
-spec handle_cast(Request :: term(), State :: term()) -> Result when
	Result :: {noreply, NewState}
			| {noreply, NewState, Timeout}
			| {noreply, NewState, hibernate}
			| {stop, Reason :: term(), NewState},
	NewState :: term(),
	Timeout :: non_neg_integer() | infinity.
%% ====================================================================
handle_cast(_Msg, State) ->
    {noreply, State}.


%% handle_info/2
%% ====================================================================
%% @doc <a href="http://www.erlang.org/doc/man/gen_server.html#Module:handle_info-2">gen_server:handle_info/2</a>
-spec handle_info(Info :: timeout | term(), State :: term()) -> Result when
	Result :: {noreply, NewState}
			| {noreply, NewState, Timeout}
			| {noreply, NewState, hibernate}
			| {stop, Reason :: term(), NewState},
	NewState :: term(),
	Timeout :: non_neg_integer() | infinity.
%% ====================================================================
handle_info(_Info, State) ->
    {noreply, State}.


%% terminate/2
%% ====================================================================
%% @doc <a href="http://www.erlang.org/doc/man/gen_server.html#Module:terminate-2">gen_server:terminate/2</a>
-spec terminate(Reason, State :: term()) -> Any :: term() when
	Reason :: normal
			| shutdown
			| {shutdown, term()}
			| term().
%% ====================================================================
terminate(_Reason, _State) ->
    ok.


%% code_change/3
%% ====================================================================
%% @doc <a href="http://www.erlang.org/doc/man/gen_server.html#Module:code_change-3">gen_server:code_change/3</a>
-spec code_change(OldVsn, State :: term(), Extra :: term()) -> Result when
	Result :: {ok, NewState :: term()} | {error, Reason :: term()},
	OldVsn :: Vsn | {down, Vsn},
	Vsn :: term().
%% ====================================================================
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.


%% ====================================================================
%% Internal functions
%% ====================================================================


