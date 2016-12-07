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
 

%% perceive_objects(+Name, +PoseAsList, +Type, +Width, +Height, +Depth, -ObjInst) is det.
%
% Perceives the objects and creates the related objects in knowrob.
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
    % needs to be modified
    create_perception_instance(Name, Perception),
    set_object_perception(ObjInst, Perception),
    set_perception_pose(Perception, Pose).
    
