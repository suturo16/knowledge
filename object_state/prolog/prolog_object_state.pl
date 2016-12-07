/** <module> prolog_object_state

@author Lukas Samel
@license BSD
*/

% defining functions
:- module(prolog_object_state,
    [
      perceive_objects/7
    ]).

% sort of importing external libraries
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_coordinates')).

%registering namespace
:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
 

%% perceive_objects(+Name, +PoseAsList, +Type, +Width, +Height, +Depth, -ObjInst) is probably det.
%
% Create the object representations in the knowledge base
% Argument 'Type' specifies perceptions classification of the object
% 
% @param Name describes the class of the object
% @param PoseAsList The pose of the object stored in a list
% @param Type The type of the object (see ObjectDetection.msg)
% @param Width The width of the object
% @param Height The height of the object
% @param Depth The depth of the object
% @param ObjInst The created object instance (optional:to be returned)
perceive_objects(Name, PoseAsList, Type, Width, Height, Depth, ObjInst) :- 
    rdf_instance_from_class(Name, ObjInst),
    create_temporal_part(Name, TemporalPart),
    set_object_temporal(ObjInst, TemporalPart),
    set_perception_pose(Perception, Pose).


%% create_temporal_part(PerceptionTypes, Perception) is det.
%
% Create a temporal part according to 4dFluents.
%
% @param Name   perception type
% @param TemporalPart    TemporalPart instance that has been created by this predicate
% 
create_temporal_part(Name, TemporalPart) :-
  atom_concat(Name,'TemporalPart', StitchedName)
  atom_concat('http://knowrob.org/kb/knowrob.owl#', StitchedName, FinalName),
  rdf_instance_from_class(FinalName, TemporalPart).

%% set_object_perception(+Object, +Perception) is det.
%
% Link the base instance to its according temporal instance
%
% @param Object        Object instance
% @param Perception    Perception instance
% 
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