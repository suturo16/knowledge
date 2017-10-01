/** <module> suturo_capabilities

@author Michael Speer 
@license BSD
*/

% defining functions
:- module(suturo_capabilities,
    [
      get_robot_with_cap_for/2,
      get_list_of_robots_with_cap/2
    ]).

:- rdf_meta get_robot_with_cap_for(r,?).

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


%registering namespace
%#:- rdf_db:rdf_register_ns(knowrob,  'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
%#:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_obj, 'package://object_state/owl/suturo_object.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_act, 'package://suturo_capabilities/owl/suturo_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(suturo_cap, 'http://knowrob.org/kb/suturo-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pepper, 'http://knowrob.org/kb/pepper.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(turtlebot, 'http://knowrob.org/kb/turtlebot.owl#', [keep(true)]).


%parse libraries
:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_map_data/owl/ccrl2_semantic_map.owl').


%% get_robot_with_cap_for(+Action, -Robot)
% MSp
% Returns robot with capabilities to do Action
get_robot_with_cap_for(Action, Robot) :-
    rdf_equal(Action,Act),
    foreach(required_cap_for_action(Act,Cap),
        cap_available_on_robot(Cap,Rob)),Robot = Rob.

%% get_robot_with_cap_for(+Action, -Robot)
% MSp
% Returns robot with capabilities for capabilities
get_robot_with_cap(Cap, Robot) :-
	cap_available_on_robot(Cap,Rob).

get_list_of_robots_with_cap(Action, Robots) :-
    bagof(required_cap_for_action(Action,Capa),nonvar(Capa),Caps),
    setof(member(Cap, Caps), cap_available_on_robot(Cap,Rob), Robots).