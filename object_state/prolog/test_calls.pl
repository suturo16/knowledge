/** <module> prolog_object_state

@author Lukas Samel, Michael Speer, Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(test_calls,
    [
      connect_frames1/1,
      connect_frames2/1,
      connect_frames4/1,
      connect_frames5/2,
      test_swrl_project/2,
      test_swrl_holds/2,
      test/1,
      test_rule_id/2,
      dummy_perception2/1,
      dummy_perception1/1,
      dummy_perception_with_close1/1,
      dummy_close/1,
      dummy_perception_with_close2/1,
      dummy_perception_with_close3/1,
      test_connect_frames/2,
      test_disconnect_frames/2
    ]).

%importing external libraries
:- use_module(library('prolog_object_state')).
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
   create_object_state_with_close(_, [[3.0,-4.0,3.0],[0.0,0.0,0.0,1.0]], Type, '/odom_combined', 2.5, 2.5, 2.5, [TimeFloat], ObjInst).


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


%% test_connect_frames(+ParentFrameID, +ChildFrameID)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
test_connect_frames(Frame1,Frame2) :-
	atom_concat('/', Frame1, Frame1Comp),
	atom_concat('/', Frame2, Frame2Comp),
	connect_frames(Frame1Comp, Frame2Comp).


%% test_disconnect_frames(+ParentFrameID, +ChildFrameID)
% LSa
% Test function for documentation. Should not be used elsewhere.
% DO NOT MODIFY - REFERENCED IN DOCUMENTARY.
test_disconnect_frames(Frame1,Frame2) :-
	atom_concat('/', Frame1, Frame1Comp),
	atom_concat('/', Frame2, Frame2Comp),
	disconnect_frames(Frame1Comp, Frame2Comp).


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


test_rule_id(Id, Descr) :-
  ( rdf_has(Descr, rdfs:label, literal(type(_,Id))) ;
    rdf_has(Descr, rdfs:label, literal(Id)) ),
  rdf_has(Descr, rdf:type, swrl:'Imp').

test(swrl_parse_rules) :-
  forall( rdf_has(Descr, rdf:type, swrl:'Imp'), (
    rdf_swrl_rule(Descr, rule(Head,Body)),
    Head \= [], Body \= []
  )).


test_swrl_holds(Id, Bindings) :-
  test_rule_id(Id, Descr),
  rdf_swrl_rule(Descr,Rule),
  swrl_vars(Rule, Vars),
  swrl_var_bindings(Vars,Bindings),
  swrl_condition_satisfied(Rule,Vars).


test_swrl_project(Id, Bindings) :-
  test_rule_id(Id, Descr),
  rdf_swrl_rule(Descr,Rule),
  swrl_vars(Rule, Vars),
  swrl_var_bindings(Vars,Bindings),
  swrl_condition_satisfied(Rule,Vars),
  swrl_implication_project(Rule,Vars).

