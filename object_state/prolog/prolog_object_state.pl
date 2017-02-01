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
      create_object_name/2,
      create_temporal_name/2,
      get_object_infos/5,
      get_object_infos/6,
      seen_since/3,
      holds_suturo/2,
      connect_frames/2,
      disconnect_frames/2,
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
      connect_frames(r,r),
      disconnect_frames(r,r),
      seen_since(r,r,r),
      holds_suturo(r,?),
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
 
%% create_object_state(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, +Begin, -ObjInst) is probably det.
% Create the object representations in the knowledge base
% Argument 'Type' specifies perceptions classification of the object
% 
% @param Name describes the class of the object
% @param PoseAsList The pose of the object stored in a list
% @param Type The type of the object (see ObjectDetection.msg)
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


create_object_state_with_close(Name, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst) :-
    ignore(close_object_state(Name)),
    create_object_state(Name, Pose, Type, Frame, Width, Height, Depth, [Begin], ObjInst).


%% close_object_state(+Name) is probably det.
%
% Closes the interval of a holding fluent 
% @param Name describes the class of the object
close_object_state(Name) :- 
    create_object_name(Name, FullName),
    owl_has(Obj,rdf:type,FullName),    
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


%% get_object_infos(+Name, -Frame, -Height, -Width, -Depth)
%
% @param Name name of the object
% @param Frame 
% @param Height
% @param Width
% @param Depth
get_object_infos(Name, Frame, Height, Width, Depth) :-
  %create_object_name(Name, FullName),
  owl_has(Obj,rdf:type,Name),
  holds_suturo(Obj, Fluent),
  owl_has(Fluent, knowrob:'frameOfObject', literal(type(xsd:string,Frame))),
  owl_has(Fluent, knowrob:'heightOfObject', literal(type(xsd:float,Height))), 
    %atom_number(HeightStr, Height),
  owl_has(Fluent, knowrob:'widthOfObject', literal(type(xsd:float,Width))),
    %atom_number(WidthStr, Width),
  owl_has(Fluent, knowrob:'depthOfObject', literal(type(xsd:float,Depth))).
    %atom_number(DepthStr, Depth).

%% get_object_infos(+Name, -FrameID, -Timestamp, -Height, -Width, -Depth)
% @TODO(Michael): Reiner Mockup --> Timestamp im Hauptcode implementieren
%
% @param Name name of the object
% @param Frame 
% @param Height
% @param Width
% @param Depth
get_object_infos(Name, FrameID, Timestamp, Height, Width, Depth) :-
  FrameID = '\cake',
  %Just give back the current time, edit this to use the time of the fluent
  current_time(Timestamp),
  Height = 30.5,
  Width = 20.1,
  Depth = 10.1.


 %% seen_since(+Name, +FrameID, +Timestamp)
 % Mockup for seen_since 
 % @TODO(Michael): implementieren
 %
 % @TODO: Params beschreiben
 %
 seen_since(Name, FrameID, Timestamp):-
   true.


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
	create_object_state_with_close(Name, Pose, 1.0, '/odom_combined', 20.0, 14.0, 9.0, Begin, ObjInst).

dummy_perception_with_close2(Name) :-
	create_object_state_with_close(Name, Pose, 1.0, '/odom_combined_a', 20.0, 14.0, 9.0, Begin, ObjInst).

dummy_close(Name) :-
	close_object_state(Name).
