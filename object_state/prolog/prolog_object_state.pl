/** <module> prolog_object_state

@author Lukas Samel, Michael Speer, Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(prolog_object_state,
    [
      create_fluent_pose/2,
      create_object_state/9,
      create_object_state/10,
      create_object_state_with_close/9,
      create_object_state_with_close/10,
      create_temporal_name/2,
      assign_obj_class/2,
      close_object_state/1,
      assert_temporal_part_with_end/3,
      connect_frames/2,
      disconnect_frames/2,
      isConnected/2,
      create_value_if_tolerance/6,
      known_object/6,
      seen_since/3,
      filter_by_odom_pos/1
    ]).

:- dynamic
        isConnected/2.

:- rdf_meta create_object_state(r,r,r,r,r,r,r,r,?),
      close_object_state(r,r,r,r,r,r,r,r,?),
      create_object_state_with_close(r,r,r,r,r,r,r,r,?),
      create_object_state_with_close(r,r,r,r,r,r,r,r,r,?),
      create_temporal_name(r,?),
      create_physical_parts(r,r),
      create_value_if_tolerance(r,r,r,r,r,r),
      assert_temporal_part_with_end(r,r,r),
      connect_frames(r,r),
      disconnect_frames(r,r),
      seen_since(r,r,r),
      filter_by_odom_pos(r),
      create_value(r,r,r,r,r).

%importing external libraries
:- use_module(library('prolog_object_state_util')).
:- use_module(library('prolog_object_state_close')).
:- use_module(library('prolog_object_state_getter_setter')).
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
:- rdf_db:rdf_register_prefix(test_swrl, 'http://knowrob.org/kb/swrl_test#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).

%parse libraries
:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_map_data/owl/ccrl2_semantic_map.owl').
:- owl_parse('package://knowrob_common/owl/swrl_test.owl').
:- owl_parse('package://object_state/owl/test_actions.owl').


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
      ; assign_obj_class(Type,ObjInst)),
    %previously used was this:
    %create_fluent(ObjInst, Fluent), fluent_assert)(S,P,O)
    (rdf_has(ObjInst, knowrob:'typeOfObject', _) ->
      true;
      rdf_assert(ObjInst, knowrob:'typeOfObject', Type)),      %# literal(type(xsd:string, Type))),
    (holds(ObjInst, knowrob:'frameOfObject', FrameID) ->
      true;
      assert_temporal_part_with_end(ObjInst, knowrob:'frameOfObject', FrameID)), %# literal(type(xsd:string, FrameID))),
    %# literal(type(xsd:float, Depth))),
    create_temporal_dimensions(ObjInst,Height,Width,Depth),
    create_fluent_pose(ObjInst, Pose),
    (rdf_has(ObjInst,knowrob:'xCoordToOdom',_) ->
      true;
      create_fluent_pose_to_odom(ObjInst, Pose)),
    (rdf_has(ObjInst,knowrob:'physicalParts',_) ->
    	true;
    	ignore(create_physical_parts(Type,ObjInst))),
    current_time(TimePoint),
    assert_temporal_part_with_end(ObjInst, knowrob:'lastPerceptionTimePoint', TimePoint).


create_object_state(Name, Pose, PoseToOdom, Type, FrameID, Width, Height, Depth, [Begin], ObjInst) :- 
    (nonvar(Name)
    -> holds(ObjInst,knowrob:'nameOfObject',Name)
      ; assign_obj_class(Type,ObjInst)),
    %previously used was this:
    %create_fluent(ObjInst, Fluent), fluent_assert)(S,P,O)
    (rdf_has(ObjInst, knowrob:'typeOfObject', _) ->
      true;
      rdf_assert(ObjInst, knowrob:'typeOfObject', Type)),      %# literal(type(xsd:string, Type))),
    (holds(ObjInst, knowrob:'frameOfObject', FrameID) ->
      true;
      assert_temporal_part_with_end(ObjInst, knowrob:'frameOfObject', FrameID)), %# literal(type(xsd:string, FrameID))),
    %# literal(type(xsd:float, Depth))),
    create_temporal_dimensions(ObjInst,Height,Width,Depth),
    create_fluent_pose(ObjInst, Pose),
    (rdf_has(ObjInst,knowrob:'xCoordToOdom',_) ->
      true;
      create_fluent_pose_to_odom(ObjInst, PoseToOdom)),
    (rdf_has(ObjInst,knowrob:'physicalParts',_) ->
      true;
      ignore(create_physical_parts(Type,ObjInst))),
    current_time(TimePoint),
    assert_temporal_part_with_end(ObjInst, knowrob:'lastPerceptionTimePoint', TimePoint).


assert_temporal_part_with_end(ObjInst, P, NewValue) :-
    current_time(Now),
    (holds(ObjInst,P,O) ->
    assert_temporal_part_end(ObjInst,P,O,Now);true),
    assert_temporal_part(ObjInst, P, NewValue). 


create_temporal_dimensions(ObjInst,Height,Width,Depth) :-
    ((holds(ObjInst, knowrob:'widthOfObject', literal(type(xsd:float, OldWidth))),    %# literal(type(xsd:float, Width))),
    holds(ObjInst, knowrob:'heightOfObject', literal(type(xsd:float, OldHeight))),  %# literal(type(xsd:float, Height))),
    holds(ObjInst, knowrob:'depthOfObject', literal(type(xsd:float, OldDepth)))) ->
    (
      current_time(Now),
      DimensionDif is 0.02,
      create_value_if_tolerance(ObjInst,knowrob:'widthOfObject',Width,OldWidth,DimensionDif,Now),
      create_value_if_tolerance(ObjInst,knowrob:'heightOfObject',Height,OldHeight,DimensionDif,Now),
      create_value_if_tolerance(ObjInst,knowrob:'depthOfObject',Depth,OldDepth,DimensionDif,Now))
    );false.

create_temporal_dimensions(ObjInst,Height,Width,Depth) :-
    ((not(holds(ObjInst, knowrob:'widthOfObject', _));    %# literal(type(xsd:float, Width))),
    not(holds(ObjInst, knowrob:'heightOfObject', _));  %# literal(type(xsd:float, Height))),
    not(holds(ObjInst, knowrob:'depthOfObject', _))) ->
    (
      assert_temporal_part(ObjInst,knowrob:'widthOfObject',Width),
      assert_temporal_part(ObjInst,knowrob:'heightOfObject',Height),
      assert_temporal_part(ObjInst,knowrob:'depthOfObject',Depth)
    );false).


%% create_object_state_with_close(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, (+)[Begin], -ObjInst)
% LSa, MSp
% Creates a fluent and closes the corresponding old TemporalPart.
create_object_state_with_close(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :-
    (known_object(Type, Pose, Width, Height, Depth, FullName),!
        -> (atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName),
          atom_concat('/', Name, ChildFrameID),
          not(isConnected(_ ,ChildFrameID))
            -> ignore(close_object_state(FullName)), 
            create_object_state(FullName, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst),!
            ; false)
        ; create_object_state(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst),!).

%% create_object_state_with_close(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, (+)[Begin], -ObjInst)
% LSa, MSp
% Creates a fluent and closes the corresponding old TemporalPart.
create_object_state_with_close(_, Pose, PoseToOdom, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :-
    known_object(Type, PoseToOdom, Width, Height, Depth, FullName)
      -> (atom_concat('http://knowrob.org/kb/knowrob.owl#', Name, FullName),
         atom_concat('/', Name, ChildFrameID),
          not(isConnected(_ ,ChildFrameID))
            -> ignore(close_object_state(FullName)),
            create_object_state(FullName, Pose, PoseToOdom, Type, Frame, Width, Height, Depth, [Begin], ObjInst),!
            ; false)
		; create_object_state(_, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst),!.


%% assign_obj_class(+Type, -ObjInst)
% MSp
% Initialized object of class from suturo_objects.owl ontology depending on object type.
assign_obj_class(Type, ObjInst) :-
    Ns = knowrob,
    get_class_name(Type, Name),
    rdf_global_id(Ns:Name, Class),
    rdf_instance_from_class(Class, ObjInst),
    % #Creates name
    multiple_objects_name(Type, NameNum), 
    create_object_name(NameNum, FullName),
    rdf_assert(ObjInst, knowrob:'nameOfObject',FullName),
    %# Adds the physicalParts. BE CAREFUL: nameOfObject for ObjInst needed
    ignore(create_physical_parts(Type,ObjInst)),!.


%Creates individuals for all physical parts
create_physical_parts(Type,ObjInst) :-
    % build knowrob:Class
    Ns = knowrob,
    get_class_name(Type, Name),
    rdf_global_id(Ns:Name, Id),
    % build parent frame id
    rdf_has(ObjInst,knowrob:'nameOfObject',ParentName),
    create_object_name(ParentNameWithoutKnowrob, ParentName),
    atom_concat('/', ParentNameWithoutKnowrob, ParentFrameID),
    % foreach physical part assert connection to object
    forall((
      rdf_has(Id,rdfs:subClassOf,A),
      rdf_has(A,owl:onProperty,knowrob:'physicalParts'),
      rdf_has(A,owl:onClass,PartClass)
    ),(
      rdf_instance_from_class(PartClass,PartInd),
      rdf_assert(ObjInst,knowrob:'physicalParts',PartInd),
      create_object_name(PhysicalPartType, PartClass),
      multiple_objects_name(PhysicalPartType, NameNum), 
      create_object_name(NameNum, FullName),
      rdf_assert(PartInd,knowrob:'nameOfObject',FullName),
      rdf_assert(PartInd,knowrob:'frameOfObject',ParentFrameID))
    ).


%% create_fluent_pose(+Fluent, +Pose)
% MSp
% @param Fluent temporal part of object
% @param Pose list of lists [[3],[4]] position and orientation
create_fluent_pose(ObjInst, [[PX, PY, PZ], [OX, OY, OZ, OW]]) :-
  (get_fluent_pose(ObjInst,[PXOld, PYOld, PZOld], [OXOld, OYOld, OZOld, OWOld]) ->
  (current_time(Now),
  PositionDif is 0.02, 
  QuaternionDiff is 0.02,
  create_value_if_tolerance(ObjInst,knowrob:'xCoord',PX,PXOld,PositionDif,Now),
  create_value_if_tolerance(ObjInst,knowrob:'yCoord',PY,PYOld,PositionDif,Now),
  create_value_if_tolerance(ObjInst,knowrob:'zCoord',PZ,PZOld,PositionDif,Now),
  ((is_different_from_by(OX,OXOld,QuaternionDiff);is_different_from_by(OZ,OZOld,QuaternionDiff);is_different_from_by(OY,OYOld,QuaternionDiff);is_different_from_by(OW,OWOld,QuaternionDiff)) ->
    (create_value(ObjInst,knowrob:'qx',OX,OXOld,Now),
    create_value(ObjInst,knowrob:'qz',OZ,OZOld,Now),
    create_value(ObjInst,knowrob:'qy',OY,OZOld,Now),
    create_value(ObjInst,knowrob:'qu',OW,OWOld,Now));
    true));
  false).


create_value_if_tolerance(ObjInst,P,NewValue,OldValue,Tolerance,Time) :-
    ((nonvar(ObjInst),nonvar(P),nonvar(NewValue),nonvar(OldValue),nonvar(Tolerance),nonvar(Time)) ->
      true;
      (debug(ObjInst),debug(P),debug(NewValue),debug(OldValue),debug(Tolerance),debug(Time)),false), %# todo: remove
    (is_different_from_by(OldValue,NewValue,Tolerance) ->
      (
        holds(ObjInst,P,Oldy), %# TODO: Why is this call needed? If i use the Value for the _end call, i get an exception
        assert_temporal_part_end(ObjInst, P,Oldy, Time),
        NewValue_=literal(type(xsd:float,NewValue)), rdf_global_term(NewValue_, NewValueV),
        assert_temporal_part(ObjInst, P, NewValueV) %# literal(type(xsd:float, PX)))
      );
        true).

create_value(ObjInst,P,NewValue,OldValue,Time) :-
        holds(ObjInst,P,Oldy), %# TODO: Why is this call needed? If i use the Value for the _end call, i get an exception
        assert_temporal_part_end(ObjInst, P,Oldy, Time),
        NewValue_=literal(type(xsd:float,NewValue)), rdf_global_term(NewValue_, NewValueV),
        assert_temporal_part(ObjInst, P, NewValueV). %# literal(type(xsd:float, PX)))


%% create_fluent_pose(+Fluent, +Pose)
% MSp
% @param Fluent temporal part of object
% @param Pose list of lists [[3],[4]] position and orientation
create_fluent_pose(ObjInst, [[PX, PY, PZ], [OX, OY, OZ, OW]]) :-
  (not(holds(ObjInst, knowrob:'xCoord', _)) ->
  (PXVal_=literal(type(xsd:'float',PX)), rdf_global_term(PXVal_, PXVal),
    assert_temporal_part(ObjInst, knowrob:'xCoord', PXVal), %# literal(type(xsd:float, PX))),
  PYVal_=literal(type(xsd:'float',PY)), rdf_global_term(PYVal_, PYVal),
    assert_temporal_part(ObjInst, knowrob:'yCoord', PYVal), %# literal(type(xsd:float, PY))),
  PZVal_=literal(type(xsd:'float',PZ)), rdf_global_term(PZVal_, PZVal),
    assert_temporal_part(ObjInst, knowrob:'zCoord', PZVal), %# literal(type(xsd:float, PZ)))
  OXVal_=literal(type(xsd:'float',OX)), rdf_global_term(OXVal_, OXVal),
    assert_temporal_part(ObjInst, knowrob:'qx', OXVal), %# literal(type(xsd:float, OX))),
  OYVal_=literal(type(xsd:'float',OY)), rdf_global_term(OYVal_, OYVal),
    assert_temporal_part(ObjInst, knowrob:'qy', OYVal), %# literal(type(xsd:float, OY))),
  OZVal_=literal(type(xsd:'float',OZ)), rdf_global_term(OZVal_, OZVal),
    assert_temporal_part(ObjInst, knowrob:'qz', OZVal), %# literal(type(xsd:float, OZ))),
  OUVal_=literal(type(xsd:'float',OW)), rdf_global_term(OUVal_, OUVal),
    assert_temporal_part(ObjInst, knowrob:'qu', OUVal)); %# literal(type(xsd:float, OW)))
  false).


%% create_fluent_pose_to_odom(+Fluent, +Pose)
% MSp
% @param Fluent temporal part of object
% @param Pose list of lists [[3],[4]] position and orientation
create_fluent_pose_to_odom(ObjInst, [[PX, PY, PZ], [OX, OY, OZ, OW]]) :-
    rdf_assert(ObjInst, knowrob:'xCoordToOdom', literal(type(xsd:float, PX))),
    rdf_assert(ObjInst, knowrob:'yCoordToOdom', literal(type(xsd:float, PY))),
    rdf_assert(ObjInst, knowrob:'zCoordToOdom', literal(type(xsd:float, PZ))),
    rdf_assert(ObjInst, knowrob:'qxToOdom', literal(type(xsd:float, OX))),
    rdf_assert(ObjInst, knowrob:'qyToOdom', literal(type(xsd:float, OY))),
    rdf_assert(ObjInst, knowrob:'qzToOdom', literal(type(xsd:float, OZ))),
    rdf_assert(ObjInst, knowrob:'quToOdom', literal(type(xsd:float, OW))).


%% close_object_state(+FullName) is probably det.
% SJo
% CAUTION: DELETES ALL OLD TEMPORALPARTS, FOR TEST ONLY
% USING THIS IN THE CREATE FUNCTION WILL MAKE THE THE RUNTIME INCREASE LINEAR (INSTEAD OF POLYNOMIAL)
% Closes the interval of a holding fluent 
% @param Name describes the class of the object
close_object_state(FullName) :-
    holds(ObjInst, knowrob:'nameOfObject', FullName),
    forall(
      (rdf_has(ObjInst,knowrob:'temporalParts',A)),
      (rdf_retractall(ObjInst,knowrob:'temporalParts',A))),!.


%% create_temporal_name(+FullName, -FullTemporalName) is det.
%
% @param FullName full object name without temporal stamp  perception 
% @returns FullTemporalName the concatenated string including the temporal stamp
create_temporal_name(FullName, FullTemporalName) :-
    atom_concat(FullName,'@t_i', FullTemporalName).


%% seen_since(+Name, +FrameID, +TimeFloat) --> true/false
%  MSp
%  @param Name name of the object in database
%  @param FrameID reference frame of object
%  @param TimeFloat timestamp to be asserted
seen_since(Name, FrameID, TimeFloat) :-
    get_object_infos(Name, FrameID, _, Timestamp, _, _, _, _),!,
    TimeFloat < Timestamp;
    close_object_state(Name).

%% known_object(+Type, +Pose, +Height, +Width, +Depth, -Name)
%MSp
% same_dimensions currently not used
known_object(Type, [Position, _], Height, Width, Depth, Name) :-
    get_object_infos_to_odom(Name, Type, [PrevPosition, _], PrevHeight, PrevWidth, PrevDepth),
    (%same_dimensions([PrevHeight, PrevWidth, PrevDepth], [Height, Width, Depth]);
	same_position(PrevPosition, Position, [Height, Width, Depth])).


%% connect_frames(+ParentFrameID, +ChildFrameID, +Pose)
% LSa
% A small function to connect two given frames.
connect_frames(ParentFrameID, ChildFrameID) :-
  prython:py_call('call_tf','get_transform',[ParentFrameID,ChildFrameID],Pose),
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


filter_by_odom_pos([[X,Y,Z],Quat]) :-
    (atom(X)->atom_number(X,XN);XN=X),
    (atom(Y)->atom_number(Y,YN);YN=Y),
    (atom(Z)->atom_number(Z,ZN);ZN=Z),
    ((ZN < 0.7) -> false;true),
    ((XN < -2.36) -> false;true).