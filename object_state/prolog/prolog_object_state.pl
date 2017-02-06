/** <module> prolog_object_state

@author Lukas Samel
@license BSD
*/

% defining functions
:- module(prolog_object_state,
    [
      create_object_state/9,
      close_object_state/1,
      create_object_state_with_close/9,
      create_fluent_pose/2,
      create_object_name/2,
      create_temporal_name/2,
      get_object_infos/5,
      get_object_infos/6,
      more_recent/1,
      seen_since/3,
      get_object_position/4,
      get_fluent_pose/3,
      holds_suturo/2,
      dummy_perception/1,
      dummy_perception_with_close/1,
      dummy_close/1,
      dummy_perception_with_close2/1
    ]).

:- rdf_meta create_object_state(r,r,r,r,r,r,r,r,?),
      close_object_state(r,r,r,r,r,r,r,r,?),
      create_object_state_with_close(r,r,r,r,r,r,r,r,?),
      create_object_name(r,?),
      create_temporal_name(r,?),
      get_object_infos(r,?,?,?,?),
      get_object_infos(r,?,?,?,?,?),
      get_object_position(r,?,?,?),
      connect_frames(r,r),
      disconnect_frames(r,r),
      seen_since(r,r,r),
      holds_suturo(r,?),
      print_shit(r),
      dummy_perception(?).

%importing external libraries
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_coordinates')).
:- use_module(library('knowrob_temporal')).
:- use_module(library('knowrob_objects')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).

%registering namespace
:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(map_obj, 'http://knowrob.org/kb/ccrl2_map_objects.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(map_obj, 'http://knowrob.org/kb/ccrl2_map_objects.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_obj, 'package://object_state/owl/suturo_object.owl#', [keep(true)]).
:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_map_data/owl/ccrl2_semantic_map.owl').
 
%% create_object_state(+Name, +Pose, +Type, +FrameID, +Width, +Height, +Depth, +Begin, -ObjInst) is probably det.
% Create the object representations in the knowledge base
% Argument 'Type' specifies perceptions classification of the object
% 
% @param Name describes the class of the object
% @param PoseAsList The pose of the object stored in a list of lists
% @param Type The type of the object (see ObjectDetection.msg)
% @param FrameID reference frame of object
% @param Width The width of the object
% @param Height The height of the object
% @param Depth The depth of the object
% @param Interval A list containing the start time and end time of a temporal
% @param ObjInst The created object instance (optional:to be returned)
create_object_state(Name, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :- 
    create_object_name(Name, FullName),
    rdf_instance_from_class(FullName, ObjInst),
    create_fluent(ObjInst, Fluent),
    %rdf_assert(Fluent, knowrob:'typeOfObject', literal(type(xsd:float, Type))),
    rdf_assert(Fluent, knowrob:'frameOfObject', literal(type(xsd:string, Frame))),
    rdf_assert(Fluent, knowrob:'widthOfObject', literal(type(xsd:float, Width))),
    rdf_assert(Fluent, knowrob:'heightOfObject',literal(type(xsd:float, Height))),
    rdf_assert(Fluent, knowrob:'depthOfObject', literal(type(xsd:float, Depth))).
    create_fluent_pose(Fluent, [Pose]).


create_object_state_with_close(Name, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :-
    ignore(close_object_state(Name)),
    create_object_state(Name, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst).

%neu MSp
create_fluent_pose(Fluent, [[PX, PY, PZ], [OX, OY, OZ, OW]]) :-
    rdf_assert(Fluent, knowrob:'xPosOfObject', literal(type(xsd:float, PX))),
    rdf_assert(Fluent, knowrob:'yPosOfObject', literal(type(xsd:float, PY))),
    rdf_assert(Fluent, knowrob:'zPosOfObject', literal(type(xsd:float, PZ))),
    rdf_assert(Fluent, knowrob:'xOriOfObject', literal(type(xsd:float, OX))),
    rdf_assert(Fluent, knowrob:'yOriOfObject', literal(type(xsd:float, OY))),
    rdf_assert(Fluent, knowrob:'zOriOfObject', literal(type(xsd:float, OZ))),
    rdf_assert(Fluent, knowrob:'wOriOfObject', literal(type(xsd:float, OW))).

%% close_object_state(+Name) is probably det.
%
% Closes the interval of a holding fluent 
% @param Name describes the class of the object
close_object_state(Name) :- 
    create_object_name(Name, FullName),
    owl_has(Obj, rdf:type,FullName),    
    fluent_assert_end(Obj,P).
    

%% create_object_name(+Name,-FullName) is det.
% Appends Name to 'http://knowrob.org/kb/knowrob.owl#'
%
% @param Name content to be appended to the namespace
% @preturns FullName the concatenated string
create_object_name(Name, FullName) :-
    atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName).


%% create_temporal_name(+FullName, -FullTemporalName) is det.
%
% @param FullName full object name without temporal stamp  perception 
% @returns FullTemporalName the concatenated string including the temporal stamp
create_temporal_name(FullName, FullTemporalName) :-
    atom_concat(FullName,'@t_i', FullTemporalName).


%% get_object_infos(+Name, -FrameID, -Height, -Width, -Depth)
%
% @param Name name of the object
% @param FrameID reference frame of object
% @param Height height of object
% @param Width width of object
% @param Depth depth of object
get_object_infos(Name, FrameID, Height, Width, Depth) :-
    %create_object_name(Name, FullName),
    owl_has(Obj,rdf:type,Name),
    holds_suturo(Obj, Fluent),
    owl_has(Fluent, knowrob:'frameOfObject', literal(type(xsd:string,FrameID))),
    owl_has(Fluent, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
      %atom_number(HeightStr, Height),
    owl_has(Fluent, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
      %atom_number(WidthStr, Width),
    owl_has(Fluent, knowrob:'depthOfObject', literal(type(xsd:float,Depth))).
      %atom_number(DepthStr, Depth).

%% get_object_infos(+Name, -FrameID, -Timestamp, -Height, -Width, -Depth)
%  MSp
% @param Name name of the object
% @param FrameID reference frame of object
% @param Timestamp start time of most recent perception
% 
get_object_infos(Name, FrameID, Timestamp, Height, Width, Depth) :-
    owl_has(Obj,rdf:type,Name),
    get_object_infos(Name, FrameID, Height, Width, Depth), 
    holds_suturo(Obj, Fluent),
    owl_has(Fluent,knowrob:'temporalExtend',I),
    owl_has(I, knowrob:'startTime', Timepoint),
    create_timepoint(Time, Timepoint),
    atom_number(Time, Timestamp).


%% seen_since(+Name, +FrameID, +Timestamp) --> true/false
%  MSp
%  @param Name name of the object in database
%  @param FrameID reference frame of object
%  @param Timestamp timestamp to be asserted
seen_since(Name, FrameID, Timestamp) :-
    owl_has(Obj,rdf:type,Name),
    holds_suturo(Obj, Fluent),
    owl_has(Fluent,knowrob:'temporalExtend',I),
    owl_has(I, knowrob:'startTime', Timepoint),
    create_timepoint(TimeStr, Timepoint),
    atom_number(TimeStr, Time),
    Time > Timestamp.


%% get_object_position(+Name, -FrameID, -Position, -Orientation)
% MSp
% @param Position position of object in frame
% @param Orientation orientation of object in frame
get_object_position(Name, FrameID, Position, Orientation) :-
    owl_has(Obj,rdf:type,Name),
    holds_suturo(Obj, Fluent),
    get_object_infos(Name, FrameID, Height, Width, Depth),
    get_fluent_pose(Fluent, Position, Orientation).

get_fluent_pose(Fluent, [PX, PY, PZ],[OX, OY, OZ, OW]) :-
    owl_has(Fluent, knowrob: 'xPosOfObject', literal(type(xsd: float, PX))),
    owl_has(Fluent, knowrob: 'yPosOfObject', literal(type(xsd: float, PY))),
    owl_has(Fluent, knowrob: 'zPosOfObject', literal(type(xsd: float, PZ))),
    owl_has(Fluent, knowrob: 'xOriOfObject', literal(type(xsd: float, OX))),
    owl_has(Fluent, knowrob: 'yOriOfObject', literal(type(xsd: float, OY))),
    owl_has(Fluent, knowrob: 'zOriOfObject', literal(type(xsd: float, OZ))),
    owl_has(Fluent, knowrob: 'wOriOfObject', literal(type(xsd: float, OW))).


%% holds_suturo(+ObjInst, -Fluent)
holds_suturo(ObjInst, Fluent) :-
    owl_has(ObjInst,knowrob:'temporalParts',Fluent),
    owl_has(Fluent,knowrob:'temporalExtend',I),
    current_time(Now),
    interval_during([Now,Now],I).

%% connect_frames(+ParentFrameID, +ChildFrameID)
%
% MOCKUP
%
connect_frames(ParentFrameID, ChildFrameID) :-
	true.

%% disconnect_frames(+ParentFrameID, +ChildFrameID)
%
% MOCKUP
%
disconnect_frames(ParentFrameID, ChildFrameID) :-
	true.

%%
% Dummy object_state
dummy_perception(Name) :-
	 create_object_state(Name, Pose, 1.0, '/odom_combined', 20.0, 14.0, 9.0, Begin, ObjInst).

dummy_perception_with_close(Name) :-
	 create_object_state_with_close(Name, Pose, 1.0, '/odom_combined', 10.0, 14.0, 9.0, Begin, ObjInst).

dummy_perception_with_close2(Name) :-
	 create_object_state_with_close(Name, Pose, 1.0, '/odom_combined_a', 5.0, 14.0, 9.0, Begin, ObjInst).

dummy_close(Name) :-
	 close_object_state(Name).
