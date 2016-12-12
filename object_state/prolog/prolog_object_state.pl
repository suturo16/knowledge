/** <module> prolog_object_state

@author Lukas Samel
@license BSD
*/

% defining functions
:- module(prolog_object_state,
    [
      create_object_state/9,
      close_object_state/9,
      create_object_name/2,
      create_temporal_name/2,
      get_object_infos/5
    ]).

% sort of importing external libraries
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_coordinates')).
:- use_module(library('knowrob_temporal')).

%registering namespace
:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
 
%% create_object_state(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, +Begin], -ObjInst) is probably det.
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
    %create_temporal_name(FullName, FullTemporalName),
    create_fluent(ObjInst, Fluent),
    rdf_assert(Fluent, knowrob:typeOfObject, literal(type(xsd:float, Type))),
    rdf_assert(Fluent, knowrob:frameOfObjekt, literal(type(xsd:string, Frame))),
    rdf_assert(Fluent, knowrob:widthOfObject, literal(type(xsd:float, Width))),
    rdf_assert(Fluent, knowrob:heightOfObject,literal(type(xsd:float, Height))),
    rdf_assert(Fluent, knowrob:depthOfObject, literal(type(xsd:float, Depth))),

    %create_temporal_part(Name, TemporalPart),
    %set_object_temporal(ObjInst, TemporalPart),
    %set_perception_pose(Perception, Pose).



%% close_object_state(+Name, +Pose, +Type, +Frame, +Width, +Height, +Depth, +[Begin,End], -ObjInst) is probably det.
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
close_object_state(Name, Pose, Type, Frame, Width, Height, Depth, [Begin,End], ObjInst) :- 
    
    fluent_assert_end(S, P).

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

  owl_has(knowrob:, knowrob:frameOfObject, literal(type(xsd:double,Frame))),
  owl_has(knowrob:, knowrob:heightOfObject, literal(type(xsd:double,Height)),
  owl_has(knowrob:, knowrob:widthOfObject, literal(type(xsd:double,Width)),
  owl_has(knowrob:, knowrob:depthOfObject, literal(type(xsd:double,Depth)),
  .
  atom_concat(Name, namespace).  

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