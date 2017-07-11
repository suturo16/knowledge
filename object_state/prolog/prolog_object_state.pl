/** <module> prolog_object_state

@author Lukas Samel, Michael Speer, Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(prolog_object_state,
    [
      assign_obj_class/2,
      close_object_state/1,
      connect_frames1/1,
      connect_frames2/1,
      connect_frames3/2,
      connect_frames4/1,
      connect_frames5/2,
      connect_frames/2,
      create_fluent_pose/2,
      create_fluent_pose_to_odom/2,
      create_object_state/9,
      create_object_state/10,
      create_object_state_with_close/9,
      create_object_state_with_close/10,
      create_object_name/2,
      create_temporal_name/2,
      disconnect_frames/2,
      dummy_perception2/1,
      dummy_perception1/1,
      dummy_perception_with_close1/1,
      dummy_close/1,
      dummy_perception_with_close2/1,
      dummy_perception_with_close3/1,
      isConnected/2,
      manually_connect_frames/2,
      manually_disconnect_frames/2,
      multiple_objects_name/2,
      get_class_name/2,
      get_fluent_pose/3,
      get_fluent_pose_to_odom/3,
      get_object_infos/5,
      get_object_infos/6,
      get_object_infos/8,
      get_object_infos/9,
      get_robot_with_cap_for/2,
      get_object_infos_to_odom/5,
      get_object_infos_to_odom/6,
      get_tf_infos/4,
      get_max_num/2,
      get_type_num/2,
      known_object/6,
      same_dimensions/2,
      same_position/3,
      seen_since/3
    ]).

:- dynamic
        isConnected/2.

:- rdf_meta create_object_state(r,r,r,r,r,r,r,r,?),
      close_object_state(r,r,r,r,r,r,r,r,?),
      create_object_state_with_close(r,r,r,r,r,r,r,r,?),
      create_object_name(r,?),
      create_temporal_name(r,?),
      get_object_infos(r,?,?,?,?),
      get_object_infos(r,?,?,?,?,?),
      get_object_infos(?,?,?,?,?,?,?,?),
      get_robot_with_cap_for(r,?),
      connect_frames(r,r),
      get_type_num(r,?),
      disconnect_frames(r,r),
      seen_since(r,r,r),
      holds_suturo(r,?),
      print_shit(r),
      dummy_perception1(?).

%importing external libraries
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
:- use_module(library('prython')).
:- use_module(library('swrl')).


%registering namespace
:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_obj, 'package://object_state/owl/suturo_object.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_act, 'package://object_state/owl/suturo_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_cap, 'http://knowrob.org/kb/suturo-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pepper, 'http://knowrob.org/kb/pepper.owl', [keep(true)]).
:- rdf_db:rdf_register_ns(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test_swrl, 'http://knowrob.org/kb/swrl_test#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).

%parse libraries
:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_map_data/owl/ccrl2_semantic_map.owl').
:- owl_parse('package://knowrob_common/owl/swrl_test.owl').
:- owl_parse('package://object_state/owl/test_actions.owl').
:- owl_parse('package://object_state/owl/suturo_objects.owl').

% Initialize python context
:- source_file(File),
   string_concat(Path,'/prolog_object_state.pl',File),
   string_concat(Path,'/../scripts',FullPath),
   add_py_path(FullPath).
 
%% create_object_state(+Name, +Pose, +Type, +FrameID, +Width, +Height, +Depth, +Begin, -ObjInst) is probably det.
% Create the object representations in the knowledge base
% Argument 'Type' specifies perceptions classification of the object
% with multiple objects the Type from perception is droped and replaced with the name, e.g. Type = cake
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
create_object_state(Name, Pose, Type, FrameID, Width, Height, Depth, [Begin], ObjInst) :- 
    (nonvar(Name)
    -> holds(ObjInst,knowrob:'nameOfObject',Name)
      ; multiple_objects_name(Type, NameNum), 
      create_object_name(NameNum, FullName),
      assign_obj_class(Type,ObjInst),
      rdf_assert(ObjInst, knowrob:'nameOfObject',FullName)),
    
    %previouslz used was this:
    %create_fluent(ObjInst, Fluent), fluent_assert)(S,P,O)
    assert_temporal_part(ObjInst, knowrob:'typeOfObject', Type),      % literal(type(xsd:string, Type))),
    assert_temporal_part(ObjInst, knowrob:'frameOfObject', FrameID),  % literal(type(xsd:string, FrameID))),
    assert_temporal_part(ObjInst, knowrob:'widthOfObject', Width),    % literal(type(xsd:float, Width))),
    assert_temporal_part(ObjInst, knowrob:'heightOfObject', Height),  % literal(type(xsd:float, Height))),
    assert_temporal_part(ObjInst, knowrob:'depthOfObject', Depth),    % literal(type(xsd:float, Depth))),
    create_fluent_pose(ObjInst, Pose),
    create_fluent_pose_to_odom(ObjInst, Pose).

create_object_state(Name, Pose, PoseToOdom, Type, FrameID, Width, Height, Depth, [Begin], ObjInst) :- 
    (nonvar(Name)
    -> holds(ObjInst,knowrob:'nameOfObject',Name)
      ; multiple_objects_name(Type, NameNum), 
      create_object_name(NameNum, FullName),
      rdf_instance_from_class(knowrob:'SpatialThing-Localized', ObjInst),
      rdf_assert(ObjInst, knowrob:'nameOfObject',FullName)),
    
    assert_temporal_part(ObjInst, knowrob:'typeOfObject', Type),      % literal(type(xsd:string, Type))),
    assert_temporal_part(ObjInst, knowrob:'frameOfObject', FrameID),  % literal(type(xsd:string, FrameID))),
    assert_temporal_part(ObjInst, knowrob:'widthOfObject', Width),    % literal(type(xsd:float, Width))),
    assert_temporal_part(ObjInst, knowrob:'heightOfObject', Height),  % literal(type(xsd:float, Height))),
    assert_temporal_part(ObjInst, knowrob:'depthOfObject', Depth),    % literal(type(xsd:float, Depth))),
    create_fluent_pose(ObjInst, Pose),
    create_fluent_pose_to_odom(ObjInst, PoseToOdom).


%% create_object_state_with_close(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, (+)[Begin], -ObjInst)
% LSa, MSp
% Creates a fluent and closes the corresponding old TemporalPart.
create_object_state_with_close(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :-
    known_object(Type, Pose, Width, Height, Depth, FullName)
        -> (atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName),
          atom_concat('/', Name, ChildFrameID),
          not(isConnected(_ ,ChildFrameID))
            -> ignore(close_object_state(FullName)),
            create_object_state(FullName, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst)
            ; false)
        ; create_object_state(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst).

%% create_object_state_with_close(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, (+)[Begin], -ObjInst)
% LSa, MSp
% Creates a fluent and closes the corresponding old TemporalPart.
create_object_state_with_close(_, Pose, PoseToOdom, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :-
    known_object(Type, PoseToOdom, Width, Height, Depth, FullName)
      -> (atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName),
         atom_concat('/', Name, ChildFrameID),
          not(isConnected(_ ,ChildFrameID))
            -> ignore(close_object_state(FullName)),
            create_object_state(FullName, Pose, PoseToOdom, Type, Frame, Width, Height, Depth, [Begin], ObjInst)
            ; false)
        ; create_object_state(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst).

%% assign_obj_class(+Type, -ObjInst)
% MSp
% Initialized object of class from suturo_objects.owl ontology depending on object type.
assign_obj_class(Type, ObjInst) :-
    get_class_name(Type, ClassName),
    create_object_name(ClassName,FullClass),
    owl_subclass_of(FullClass,knowrob:'SpatialThing-Localized')
    -> rdf_instance_from_class(FullClass, ObjInst);
    rdf_instance_from_class(knowrob:'SpatialThing-Localized', ObjInst).


%% get_class_name(+Type, -ClassName)
% MSp
% converts first letter of Type into capital letter
get_class_name(Type, ClassName) :-
    sub_atom(Type,0,1,_,C), char_code(C,I), 96<I, I<123
    -> J is I-32, char_code(D,J), sub_atom(Type,1,_,0,Sub), atom_concat(D,Sub,ClassName)
    ; ClassName = Type.


%% create_fluent_pose(+Fluent, +Pose)
% MSp
% @param Fluent temporal part of object
% @param Pose list of lists [[3],[4]] position and orientation
create_fluent_pose(ObjInst, [[PX, PY, PZ], [OX, OY, OZ, OW]]) :-
	PXVal_=literal(type(xsd:'float',PX)), rdf_global_term(PXVal_, PXVal),
    assert_temporal_part(ObjInst, knowrob:'xCoord', PXVal), % literal(type(xsd:float, PX))),
	PYVal_=literal(type(xsd:'float',PY)), rdf_global_term(PYVal_, PYVal),
    assert_temporal_part(ObjInst, knowrob:'yCoord', PYVal), % literal(type(xsd:float, PY))),
	PZVal_=literal(type(xsd:'float',PZ)), rdf_global_term(PZVal_, PZVal),
    assert_temporal_part(ObjInst, knowrob:'zCoord', PZVal), % literal(type(xsd:float, PZ))),
	OXVal_=literal(type(xsd:'float',OX)), rdf_global_term(OXVal_, OXVal),
    assert_temporal_part(ObjInst, knowrob:'qx', OXVal), % literal(type(xsd:float, OX))),
	OYVal_=literal(type(xsd:'float',OY)), rdf_global_term(OYVal_, OYVal),
    assert_temporal_part(ObjInst, knowrob:'qy', OYVal), % literal(type(xsd:float, OY))),
	OZVal_=literal(type(xsd:'float',OZ)), rdf_global_term(OZVal_, OZVal),
    assert_temporal_part(ObjInst, knowrob:'qz', OZVal), % literal(type(xsd:float, OZ))),
	OUVal_=literal(type(xsd:'float',OW)), rdf_global_term(OUVal_, OUVal),
    assert_temporal_part(ObjInst, knowrob:'qu', OUVal). % literal(type(xsd:float, OW))).

%% create_fluent_pose_to_odom(+Fluent, +Pose)
% MSp
% @param Fluent temporal part of object
% @param Pose list of lists [[3],[4]] position and orientation
create_fluent_pose_to_odom(ObjInst, [[PX, PY, PZ], [OX, OY, OZ, OW]]) :-
    PXVal_=literal(type(xsd:'float',PX)), rdf_global_term(PXVal_, PXVal),
    assert_temporal_part(ObjInst, knowrob:'xCoordToOdom', PXVal), % literal(type(xsd:float, PX))),
	PYVal_=literal(type(xsd:'float',PY)), rdf_global_term(PYVal_, PYVal),
    assert_temporal_part(ObjInst, knowrob:'yCoordToOdom', PYVal), % literal(type(xsd:float, PY))),
	PZVal_=literal(type(xsd:'float',PZ)), rdf_global_term(PZVal_, PZVal),
    assert_temporal_part(ObjInst, knowrob:'zCoordToOdom', PZVal), % literal(type(xsd:float, PZ))),
	OXVal_=literal(type(xsd:'float',OX)), rdf_global_term(OXVal_, OXVal),
    assert_temporal_part(ObjInst, knowrob:'qxToOdom', OXVal), % literal(type(xsd:float, OX))),
	OYVal_=literal(type(xsd:'float',OY)), rdf_global_term(OYVal_, OYVal),
    assert_temporal_part(ObjInst, knowrob:'qyToOdom', OYVal), % literal(type(xsd:float, OY))),
	OZVal_=literal(type(xsd:'float',OZ)), rdf_global_term(OZVal_, OZVal),
    assert_temporal_part(ObjInst, knowrob:'qzToOdom', OZVal), % literal(type(xsd:float, OZ))),
	OUVal_=literal(type(xsd:'float',OW)), rdf_global_term(OUVal_, OUVal),
    assert_temporal_part(ObjInst, knowrob:'quToOdom', OUVal). % literal(type(xsd:float, OW))).

%% close_object_state(+FullName) is probably det.
% SJo
% Closes the interval of a holding fluent 
% @param Name describes the class of the object
close_object_state(FullName) :-
    holds(ObjInst, knowrob:'nameOfObject', FullName),
    % FIXME: Should be replaced by fluent_assert_end if it works.
    current_time(Now),
    forall((holds(ObjInst,P,O),O \= FullName), 
    assert_temporal_part_end(ObjInst, P, O, Now)),!.

%    rdf_has(ObjInst, knowrob:'temporalParts',SubjectPart),
%    rdf_has(SubjectPart, P, _),
%    rdf_has(SubjectPart, knowrob:'temporalExtend', I),
%    not( rdf_has(I, knowrob:'endTime', _) ),
%    create_timepoint(Now, IntervalEnd),
%    rdf_assert(I, knowrob:'endTime', IntervalEnd).
%    fluent_assert_end(Obj,P).
    

%% multiple_objects_name(+Type, -NameNum)
% MSp
% creates object name depending on how many other objects of same type exist
% @param Name type leading to NameNum by appending iterating number
% @param NameNum is the unique name of the new object in KB, e.g. cake32
multiple_objects_name(Type, NameNum) :-
    (get_max_num(Type, Num), number(Num)
    -> Number is Num+1; Number is 1),
    atom_concat(Type, Number, NameNum).


%% get_max_num(+Type, - Number)
% MSp
% gets maximum number from name of object type
get_max_num(Type, Number) :-
    get_type_num(Type, NumA), get_type_num(Type, NumB), NumA > NumB
    -> get_type_num(Type, Number), get_type_num(Type, NumC), Number > NumC;
      get_type_num(Type, Number).


%% get_type_num(+Type, -Number)
% MSp
% helper funciton for to get max Number in NameNum
get_type_num(Type, Number) :-
    get_object_infos(FullName,_,Type,_,_,_,_,_), 
    create_object_name(NameNum,FullName),
    atom_concat(Type, NumChar, NameNum), 
    atom_number(NumChar, Number).


%% strip_name_num(+NameNum, -Name)
% MSp
% removes appended numbers from object name
% @param NameNum is the name with appended type count, e.g. cake32
% @param Name is the type of the object
strip_name_num(NameNum, Name) :-
    sub_atom(NameNum, _,1,0,L), atom_number(L, _)
    -> sub_string(NameNum,0,_,1, Sub), strip_name_num(Sub, Name);
      Name = NameNum.


%% create_object_name(+Name,-FullName) is det.
% LSa
% Appends Name to the knowrob domain.
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
    holds(Obj, knowrob:'typeOfObject', Type),       % literal(type(xsd:string,Type))),
    holds(Obj,knowrob:'nameOfObject', Name),
    holds(Obj, knowrob:'frameOfObject', FrameID),   % literal(type(xsd:string,FrameID))),
    holds(Obj, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
    holds(Obj, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
    holds(Obj, knowrob:'depthOfObject', literal(type(xsd:float,Depth))),
    once(get_fluent_pose(Obj, Position, Orientation)),
   	ignore((
    current_time(Now),
    interval_during(Now, Interval),
    interval_start(Interval, Timestamp))).

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

%% seen_since(+Name, +FrameID, +TimeFloat) --> true/false
%  MSp
%  @param Name name of the object in database
%  @param FrameID reference frame of object
%  @param TimeFloat timestamp to be asserted
seen_since(Name, FrameID, TimeFloat) :-
    get_object_infos(Name, FrameID, _, Timestamp, _, _, _, _),
    TimeFloat < Timestamp;
    close_object_state(Name).


%% get_tf_infos(-Name, -FrameID, -Position, -Orientation)
% LSa
% A function to bundle the required information for the TF-broadcaster.
% @param Name 
% @param FrameID 
% @param Position position of object in frame
% @param Orientation orientation of object in frame
get_tf_infos(Name, FrameID, Position, Orientation) :-
    holds(Obj,knowrob:'nameOfObject',FullName),
    create_object_name(Name, FullName),
    holds(Obj, knowrob:'frameOfObject', FrameID),   % literal(type(xsd:string,FrameID))),
    get_fluent_pose(Obj, Position, Orientation).


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

%% known_object(+Type, +Pose, +Height, +Width, +Depth, -Name)
%MSp
% same_dimensions currently not used
% CHANGED, UNTESTED:  if frame differs, transform and proceed
%           otherwise do as usual
known_object(Type, [Position, _], Height, Width, Depth, Name) :-
    get_object_infos_to_odom(Name, Type, [PrevPosition, _], PrevHeight, PrevWidth, PrevDepth),
    (%same_dimensions([PrevHeight, PrevWidth, PrevDepth], [Height, Width, Depth]);
    same_position(PrevPosition, Position, [Height, Width, Depth])).

%% same_dimensions(+[PrevDim], +[CurDim])
%MSp
%@param PrevDim dimensions of object at previous timestamp
%@param CurDim  dimensions of object at current  timestamp
same_dimensions([E|PrevDim], CurDim) :-
    length(PrevDim, 0) -> member(E, CurDim);
    same_dimensions([E], CurDim), same_dimensions(PrevDim, CurDim).


%% same_position(+[PrevPos], +[CurPos], +[Dimensions])
%MSp
%@param PrevPos position of object at previous timestamp
%@param CurPos  position of object at current  timestamp
%@param Dimensions tolerance allowed between positions to consider unchanged is maximum dimension of object
same_position(PrevPos, CurPos, Dimensions) :-
    euclidean_dist(PrevPos, CurPos, Dist),
    max_member(Dmax, Dimensions),
    Dist < Dmax.
    

%% euclidean_dist(+[PointA], +[PointB], -Dist)
% MSp
euclidean_dist(PointA, PointB, Dist) :-
    length(PointA, Len), length(PointB, Len) ->
    sqr_sum(PointA, PointB, SqrSum), Dist is sqrt(SqrSum); false.


%% sqr_sum(+[A1|An], +[B1|Bn], -SqrSum)
% MSp
sqr_sum([A1|An], [B1|Bn], SqrSum) :-
    length(An, 0) ->
    SqrSum is ((A1-B1)^2.0);
    sqr_sum(An, Bn, Sum),
    SqrSum is Sum + ((A1-B1)^2.0).


%% get_robot_with_cap_for(+Action, -Robot)
% MSp
% Returns robot with capabilities to do Action
get_robot_with_cap_for(Action, Robot) :-
    rdf_equal(Action,Act),
    foreach(required_cap_for_action(Act,Cap),
        cap_available_on_robot(Cap,Rob)),Robot = Rob.

get_list_of_robots_with_cap(Action, Robots) :-
    bagof(required_cap_for_action(Action,Capa),nonvar(Capa),Caps),
    setof(member(Cap, Caps), cap_available_on_robot(Cap,Rob), Robots).


%% connect_frames(+ParentFrameID, +ChildFrameID, +Pose)
% LSa
% A small function to connect two given frames.
connect_frames(ParentFrameID, ChildFrameID) :-
  prython:py_call('call_tf','get_transform',[ParentFrameID,ChildFrameID],Pose),
  write(Pose),
  atom_concat('/', Name, ChildFrameID),
  atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName),
  get_object_infos_to_odom(FullName, Type, PoseToOdom, Height, Width, Depth),
  create_object_state_with_close(Name, Pose, PoseToOdom, Type, ParentFrameID, Width, Height, Depth, [Begin], ObjInst),
  assert(isConnected(ParentFrameID, ChildFrameID)).

%% disconnect_frames(+ParentFrameID, +ChildFrameID)
% LSa
% A simple function to disconnect two given frames.
disconnect_frames(ParentFrameID, ChildFrameID) :-
  retract(isConnected(ParentFrameID, ChildFrameID)).

%%
% Dummy object_state
dummy_perception1(Type) :-
   % atom_concat(Type, '1', Name),
   get_time(TimeFloat),
	 create_object_state(_, [[5.0,4.0,3.0],[6.0,7.0,8.0,1.0]], Type, '/odom_combined', 1.0, 1.0, 1.0, [TimeFloat], ObjInst).


dummy_perception2(Type) :-
   % atom_concat(Type, '2', Name),
   get_time(TimeFloat),
   create_object_state(_, [[15.0,14.0,13.0],[0.0,0.0,0.0,1.0]], Type, '/odom_combined', 2.0, 4.0, 9.0, [TimeFloat], ObjInst).

dummy_perception_with_close1(Type) :-
   % atom_concat(Type, '1', Name),
   get_time(TimeFloat),
	 create_object_state_with_close(_, [[1.0,1.0,1.0],[0.0,0.0,0.0,1.0]], Type, '/odom_combined', 2.0, 2.0, 2.0, [TimeFloat], ObjInst).

dummy_perception_with_close2(Type) :-
   % atom_concat(Type, '2', Name),
   get_time(TimeFloat),
   create_object_state_with_close(_, [[1.0,1.0,1.0],[0.0,0.0,0.0,1.0]], Type, '/odom_combined', 2.5, 2.5, 2.5, [TimeFloat], ObjInst).

dummy_perception_with_close3(Type) :-
   % atom_concat(Type, '2', Name),
   get_time(TimeFloat),
   create_object_state_with_close(_, [[5.0,-1.0,3.0],[0.0,0.0,0.0,1.0]], Type, '/odom_combined', 2.0, 1.0, 2.0, [TimeFloat], ObjInst).


dummy_close(Name) :-
  create_object_name(Name,FullName),
	close_object_state(FullName).

% Dummy object_state
dummy_perception2(Egal) :-
   create_object_state_with_close('carrot1', [[5.0,4.0,3.0],[6.0,7.0,8.0,9.0]], 1.0, '/odom_combined', 20.0, 14.0, 9.0, Begin, ObjInst).

%% connect_frames1(+Name)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
connect_frames1(Name) :-
create_object_state(Name, [[5.0,4.0,3.0],[6.0,7.0,8.0,9.0]], 1.0, '/odom_combined', 20.0, 14.0, 9.0, Begin, ObjInst).

%% connect_frames2(+Name)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
connect_frames2(Name) :-
   create_object_state_with_close(Name, [[5.0,4.0,3.0],[6.0,7.0,8.0,9.0]], 1.0, '/odom_combined', 20.0, 14.0, 9.0, Begin, ObjInst).

%% connect_frames3(+ParentFrameID, +ChildFrameID)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
connect_frames3(ParentFrameID, ChildFrameID) :-
   connect_frames('/baum', '/table').

%% manually_connect_frames(+ParentFrame, +ChildFrame)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
manually_connect_frames(ParentFrame, ChildFrame) :-
  atom_concat('/', ParentFrame, ParentFrameStitched),
  atom_concat('/', ChildFrame, ChildFrameStitched),
  connect_frames(ParentFrameStitched, ChildFrameStitched).

%% manually_disconnect_frames(+ParentFrame, +ChildFrame)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
manually_disconnect_frames(ParentFrame, ChildFrame) :-
  atom_concat('/', ParentFrame, ParentFrameStitched),
  atom_concat('/', ChildFrame, ChildFrameStitched),
  disconnect_frames(ParentFrameStitched, ChildFrameStitched).

%% connect_frames4(+Name)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
connect_frames4(Name) :-
   create_object_state_with_close(Name, [[8.0,7.0,7.0],[6.0,7.0,8.0,9.0]], 1.0, '/odom_combined', 20.0, 14.0, 9.0, Begin, ObjInst).

%% connect_frames5(+ParentFrameID, +ChildFrameID)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
connect_frames5(ParentFrameID, ChildFrameID) :-
   atom_concat('/', ParentFrameID, UsableParentFrameID),
   atom_concat('/', ChildFrameID, UsableChildFrameID),
   connect_frames(UsableParentFrameID, UsableChildFrameID, [[8.0,7.0,7.0],[6.0,7.0,8.0,9.0]]).
