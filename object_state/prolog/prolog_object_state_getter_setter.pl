/** <module> prolog_object_state_getter_setter

@author Lukas Samel, Michael Speer, Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(prolog_object_state_getter_setter,
    [
      get_info/2,
      get_info/3,
      get_object/2,
      set_info/2,
      get_object_infos/5,
      get_object_infos/6,
      get_object_infos/8,
      get_object_infos/9,
      get_object_infos_to_odom/5,
      get_object_infos_to_odom/6,
      get_fluent_pose/3,
      get_fluent_pose_to_odom/3,
      get_physical_parts/3,
      get_tf_infos/4
    ]).

:- dynamic
        isConnected/2.

:- rdf_meta set_info(r,r),
      set_info(r,r,r),
      get_info(r,r),
      get_info(r,r,?),
      get_object_infos(r,?,?,?,?),
      get_object_infos(r,?,?,?,?,?),
      get_object_infos(?,?,?,?,?,?,?,?),
      get_object_infos(?,?,?,?,?,?,?,?,?),
      get_physical_parts(r,r,r).

%importing external libraries
:- use_module(library('prolog_object_state_util')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_coordinates')).
:- use_module(library('knowrob_temporal')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_owl')).
:- use_module(library('srdl2')).
:- use_module(library('rdfs_computable')).


%registering namespace
%#:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_obj, 'package://object_state/owl/suturo_object.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_act, 'package://object_state/owl/suturo_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_cap, 'http://knowrob.org/kb/suturo-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pepper, 'http://knowrob.org/kb/pepper.owl', [keep(true)]).
:- rdf_db:rdf_register_ns(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).


%parse libraries
:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_map_data/owl/ccrl2_semantic_map.owl').
:- owl_parse('package://knowrob_common/owl/swrl_test.owl').
:- owl_parse('package://object_state/owl/test_actions.owl').

%%################################################  NEU  ###################################################

set_info(Thing, Info) :-
    not(rdf_has(Thing, rdf:type, _)),(
    (
      % #if specified by name update object
      ( rdf_global_id(knowrob:Thing,Id),
      holds(ObjInst, knowrob:nameOfObject, Id)
      ; holds(ObjInst, knowrob:nameOfObject, Thing)  ),
      set_info(ObjInst, Info)
    );(
      % #if Thing is instance of class in knowrob
      owl_subclass_of(Id, knowrob:'SpatialThing'), 
      rdf_global_id(knowrob:Thing,Id), 
      assign_obj_class(Thing, ObjInst),
      assert_temporal_part(ObjInst, knowrob:typeOfObject, Thing),
      set_info(ObjInst, Info)
    )
    % #if Thing is not specified but can be uniquely identified
    % #TODO check and uncomment if required
    % #setof(Obj, (setof(X, (member(X, Info), is_list(X)), Conds),
    % #           get_object(Obj)), Objs),
    % #length(Objs, 1), [ObjInst|_] = Objs,
    % #set_info(ObjInst, Info).
    ),!.

%% #set_info(ObjInst, [[P,Val]|More])
% #MSp
% #saves knowledge value O as relation P to S
% @param ObjInst the target object or thing for the P related value Val
% @param P the relation the value Val has to subject S
% @param Val the value of the P related object
set_info(ObjInst, [[P,Val]|More]) :-
    rdf_has(ObjInst, rdf:type, _), % #check that ObjInst is object
    set_info(ObjInst, More)
    -> 
      ( nonvar(P), nonvar(Val)       % #P & Val can't be variables -> if so skip
      -> ( Ns = knowrob,
        rdf_global_id(Ns:P, Id),
        ignore(forall( holds(ObjInst, Id, EndVal),
        assert_temporal_part_end(ObjInst, Id, EndVal))),
        assert_temporal_part(ObjInst, Id, Val),
        set_info(ObjInst, More),! )
      ; true )
    ;false .

set_info(_, []).

%% get_info(+Variables, -Returns)
get_info(Variables, Returns) :-
    is_list(Variables), var(Returns),
    findall(X, (member(X, Variables), is_list(X)), Conds),
    findall(Y, (member(Y, Variables), not(is_list(Y))), Vars),
    get_object(Conds, ObjInst),
    (not(length(Vars,0)), get_info(Vars, Returns, ObjInst);
      length(Vars,0), get_info(ObjInst, Returns)).

get_info(ObjName, Returns) :-
    not(is_list(ObjName)), var(Returns), Ns = knowrob, 
    ( rdf_global_id(Ns:ObjName, ObjInst),
    rdf_has(ObjInst, rdf:type, _)
    ; rdf_global_id(Ns:ObjName, Name),
    holds(ObjInst, knowrob:nameOfObject, Name)  ),
    get_info(ObjInst, Returns),!.

get_info(ObjInst, Returns) :-
    not(is_list(ObjInst)), var(Returns), 
    % #check that ObjInst is object:
    rdf_has(ObjInst, rdf:type, _),  
    Ns = knowrob,
    findall([Var,Val], (
      holds(ObjInst, NsVar, RDFvalue),
      rdf_global_id(Ns:Var, NsVar),
      once(
        rdf_global_id(_:Val, RDFvalue); strip_literal_type(RDFvalue, Val))),
      Returns),!.

%% get_info(Vars, Rets, ObjInst)
% MSp
get_info(Variables,Returns, ObjInst) :-
	not(is_list(ObjInst)),var(Returns),
    Ns = knowrob,
    findall([Var, Val], (
      member(Var, Variables),
      holds(ObjInst, NsVar, RDFvalue),
      rdf_global_id(Ns:Var, NsVar),
      once(
        rdf_global_id(_:Val, RDFvalue); strip_literal_type(RDFvalue, Val))),
      Returns),!.


%% get_object(+Conditions, -ObjInst)
% MSp
% Helper to identify a certain Object according to given Conditions
get_object([[Cond,Val]|Conds], ObjInst) :-
    Ns = knowrob,
    rdf_global_id(Ns:Cond, NsVar),
    holds(ObjInst, NsVar, RDFvalue), 
    once( 
      rdf_global_id(_:Val, RDFvalue) ; strip_literal_type(RDFvalue, Val)),
    get_object(Conds, ObjInst).

get_object([], _).

%%################################################  Alt  ###################################################

 
get_object_infos(Name, FrameID, Height, Width, Depth) :-
    get_object_infos(Name, FrameID, _, _, _, Height, Width, Depth,_).


get_object_infos(Name, FrameID, Timestamp, Height, Width, Depth) :-
    get_object_infos(Name, FrameID, _, Timestamp, _, Height, Width, Depth,_).


get_object_infos(Name, FrameID, Type, Timestamp, [Position, Orientation], Height, Width, Depth) :-
    get_object_infos(Name, FrameID, Type, Timestamp, [Position, Orientation], Height, Width, Depth,_).


%% get_object_infos(+Name, -FrameID, -Type, -Timestamp, -Pose, -Height, -Width, -Depth, -Obj)
% LSa, MSp
% @param Name name of the object
% @param FrameID reference frame of object pose
% @param Timestamp start time of most recent perception
% @param Type type of the object
% @param Pose list of [Position, Orientation] of object
% @param Height height of object
% @param Width width of object
% @param Depth depth of object
% @param Obj object ID in KB
get_object_infos(Name, FrameID, Type, Timestamp, [Position, Orientation], Height, Width, Depth, Obj) :-
    rdf_has(Obj,knowrob:'nameOfObject', Name),
    rdf_has(Obj, knowrob:'typeOfObject', Type),       % literal(type(xsd:string,Type))),
    holds(Obj, knowrob:'frameOfObject', FrameIDLit),
    strip_literal_type(FrameIDLit,FrameID),
    holds(Obj, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
    holds(Obj, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
    holds(Obj, knowrob:'depthOfObject', literal(type(xsd:float,Depth))),
    get_fluent_pose(Obj, Position, Orientation),
    holds(Obj, knowrob:'lastPerceptionTimePoint',literal(type(xsd:float,Timestamp))).


%% get_fluent_pose(Object, [PX, PY, PZ],[OX, OY, OZ, OW])
% MSp
get_fluent_pose(Object, [PX, PY, PZ],[OX, OY, OZ, OW]) :-
    holds(Object, knowrob:'xCoord', literal(type(xsd: float, PX))),
    holds(Object, knowrob:'yCoord', literal(type(xsd: float, PY))),
    holds(Object, knowrob:'zCoord', literal(type(xsd: float, PZ))),
    holds(Object, knowrob:'qx', literal(type(xsd: float, OX))),
    holds(Object, knowrob:'qy', literal(type(xsd: float, OY))),
    holds(Object, knowrob:'qz', literal(type(xsd: float, OZ))),
    holds(Object, knowrob:'qu', literal(type(xsd: float, OW))).


get_object_infos_to_odom(Type, [Position, Orientation], Height, Width, Depth) :-
    holds(Obj, knowrob:'typeOfObject', Type),
    holds(Obj, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
    holds(Obj, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
    holds(Obj, knowrob:'depthOfObject', literal(type(xsd:float,Depth))),
    get_fluent_pose_to_odom(Obj, Position, Orientation).


get_object_infos_to_odom(Name, Type, [Position, Orientation], Height, Width, Depth) :-
    holds(Obj,knowrob:'nameOfObject',Name),
    holds(Obj, knowrob:'typeOfObject', Type),
    holds(Obj, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
    holds(Obj, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
    holds(Obj, knowrob:'depthOfObject', literal(type(xsd:float,Depth))),
    get_fluent_pose_to_odom(Obj, Position, Orientation).


%% get_fluent_pose_to_odom(Object, [PX, PY, PZ],[OX, OY, OZ, OW])
% MSp
get_fluent_pose_to_odom(Object, [PX, PY, PZ],[OX, OY, OZ, OW]) :-
    holds(Object, knowrob:'xCoordToOdom', literal(type(xsd: float, PX))),
    holds(Object, knowrob:'yCoordToOdom', literal(type(xsd: float, PY))),
    holds(Object, knowrob:'zCoordToOdom', literal(type(xsd: float, PZ))),
    holds(Object, knowrob:'qxToOdom', literal(type(xsd: float, OX))),
    holds(Object, knowrob:'qyToOdom', literal(type(xsd: float, OY))),
    holds(Object, knowrob:'qzToOdom', literal(type(xsd: float, OZ))),
    holds(Object, knowrob:'quToOdom', literal(type(xsd: float, OW))).


get_physical_parts(Name, PhysicalParts, PhysicalPartName) :-
    (rdf_has(Obj,knowrob:'nameOfObject', Name),!),
    holds(Obj,knowrob:'physicalParts',PhysicalParts),
    rdf_has(PhysicalParts,knowrob:'nameOfObject', PhysicalPartName).


%% get_fluent_pose(Object, [PX, PY, PZ],[OX, OY, OZ, OW])
% MSp
get_pose(Object, [PX, PY, PZ],[OX, OY, OZ, OW]) :-
    owl_has(Object, knowrob:'xCoord', literal(type(xsd: float, PXOld))),
    (atom(PXOld) -> atom_number(PXOld,PX);PX=PXOld),
    owl_has(Object, knowrob:'yCoord', literal(type(xsd: float, PYOld))),
    (atom(PYOld) -> atom_number(PYOld,PY);PY=PYOld),
    owl_has(Object, knowrob:'zCoord', literal(type(xsd: float, PZOld))),
    (atom(PZOld) -> atom_number(PZOld,PZ);PZ=PZOld),
    owl_has(Object, knowrob:'qx', literal(type(xsd: float, OXOld))),
    (atom(OXOld) -> atom_number(OXOld,OX);OX=OXOld),
    owl_has(Object, knowrob:'qy', literal(type(xsd: float, OYOld))),
    (atom(OYOld) -> atom_number(OYOld,OY);OY=OYOld),
    owl_has(Object, knowrob:'qz', literal(type(xsd: float, OZOld))),
    (atom(OZOld) -> atom_number(OZOld,OZ);OZ=OZOld),
    owl_has(Object, knowrob:'qu', literal(type(xsd: float, OWOld))),
    (atom(OWOld) -> atom_number(OWOld,OW);OW=OWOld).


%% get_tf_infos(-Name, -FrameID, -Position, -Orientation)
% LSa
% A function to bundle the required information for the TF-broadcaster.
% @param Name 
% @param FrameID 
% @param Position position of object in frame
% @param Orientation orientation of object in frame
get_tf_infos(Name, StrippedFrameID, Position, Orientation) :-
    holds(Obj,knowrob:'nameOfObject',FullName),
    strip_literal_type(FullName, StrippedFullName),
    create_object_name(Name, StrippedFullName),
    holds(Obj, knowrob:'frameOfObject', FrameID),   % literal(type(xsd:string,FrameID))),
    strip_literal_type(FrameID, StrippedFrameID),
    (holds(Obj, knowrob:'xCoord', _) -> 
      get_fluent_pose(Obj, Position, Orientation); 
      get_pose(Obj, Position, Orientation)).

