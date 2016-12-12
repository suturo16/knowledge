/** <module> prolog_object_state

@author Lukas Samel
@license BSD
*/

% defining functions
:- module(prolog_object_state,
    [
      perceive_objects/9
    ]).

% sort of importing external libraries
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
 

%% perceive_objects(+Name, +PoseAsList, +Type, +Frame, +Width, +Height, +Depth, +Interval, -ObjInst) is probably det.
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
perceive_objects(Name, PoseAsList, Type, Frame, Width, Height, Depth, [Begin,End], ObjInst) :- 
	true.


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

%% set_object_perception(+Object, +Perception) is det.
% Link the base instance to its according temporal instance
%
% @param Object        Object instance
% @param Perception    Perception instance
set_object_perception(Object, Perception) :-
  rdf_assert(TemporalPart, knowrob:temporalPartOf, ObjInst).

  % add perception to linked list of object detections,
  %((rdf_has(Object, knowrob:latestDetectionOfObject, Prev)) -> (
  %
  %   rdf_update(Object, knowrob:latestDetectionOfObject, Prev, object(Perception)),
  %   rdf_assert(Perception, knowrob:previousDetectionOfObject, Prev)
  %) ; (
  %  rdf_assert(TemporalPart, knowrob:temporalPartOf, ObjInst),
  %)),
  % update latestDetectionOfObject pointer to list head
  %rdf_assert(Perception, knowrob:objectActedOn, Object).
