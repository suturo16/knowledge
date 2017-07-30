
%% This file contains tests for the prolog object state module
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.

:- begin_tests(prolog_object_state).

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


test(create_with_change_in_position) :-
	create_object_state_with_close(_, [[1.0,1.0,1.0],[0.0,0.0,0.0,1.0]], 'cake', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
	(rdf_equal(knowrob:'cake1',Name),get_object_infos(Name, '/odom_combined', cake, _, [[1.0,1.0,1.0], [0.0,0.0,0.0,1.0]], 2.0, 2.0, 2.0),!),
	loop(100),
	(rdf_equal(knowrob:'cake1',Name),get_object_infos(Name, '/odom_combined', cake, _, [[1.0,1.0,1.0], [0.0,0.0,0.0,1.0]], 2.0, 2.0, 2.0),!),
	create_object_state_with_close(_, [[2.0,1.0,1.0],[0.0,0.0,0.0,1.0]], 'cake', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
	(rdf_equal(knowrob:'cake1',Name),get_object_infos(Name, '/odom_combined', cake, _, [[2.0,1.0,1.0], [0.0,0.0,0.0,1.0]], 2.0, 2.0, 2.0),!).

test(change_of_dimensions) :-
	create_object_state_with_close(_, [[-13.0,-12.0,-18.0],[0.0,0.0,0.0,1.0]], 'box', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
	(rdf_equal(knowrob:'box1',Name),get_object_infos(Name, '/odom_combined', box, _, [[-13.0,-12.0,-18.0], [0.0,0.0,0.0,1.0]], 2.0, 2.0, 2.0),!),
	create_object_state_with_close(_, [[-13.0,-12.0,-18.0],[0.0,0.0,0.0,1.0]], 'box', '/odom_combined', 3.0, 2.0, 2.0, [_], _),
	(rdf_equal(knowrob:'box1',Name),get_object_infos(Name, '/odom_combined', box, _, [[-13.0,-12.0,-18.0], [0.0,0.0,0.0,1.0]], 2.0, 3.0, 2.0),!),
	create_object_state_with_close(_, [[-13.0,-12.0,-18.0],[0.0,0.0,0.0,1.0]], 'box', '/odom_combined', 3.0, 3.0, 2.0, [_], _),
	(rdf_equal(knowrob:'box1',Name),get_object_infos(Name, '/odom_combined', box, _, [[-13.0,-12.0,-18.0], [0.0,0.0,0.0,1.0]], 3.0, 3.0, 2.0),!),
	create_object_state_with_close(_, [[-13.0,-12.0,-18.0],[0.0,0.0,0.0,1.0]], 'box', '/odom_combined', 3.0, 3.0, 5.0, [_], _),
	(rdf_equal(knowrob:'box1',Name),get_object_infos(Name, '/odom_combined', box, _, [[-13.0,-12.0,-18.0], [0.0,0.0,0.0,1.0]], 3.0, 3.0, 5.0),!).


test(change_of_frameid) :-
	create_object_state_with_close(_, [[-83.0,12.0,80.0],[0.0,0.0,0.0,1.0]], 'can', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
	(rdf_equal(knowrob:'can1',Name),get_object_infos(Name, '/odom_combined', can, _, [[-83.0,12.0,80.0], [0.0,0.0,0.0,1.0]], 2.0, 2.0, 2.0),!),
	create_object_state_with_close(_, [[-83.0,12.0,80.0],[0.0,0.0,0.0,1.0]], 'can', '/map', 2.0, 2.0, 2.0, [_], _),
	(rdf_equal(knowrob:'can1',Name),get_object_infos(Name, '/map', can, _, [[-83.0,12.0,80.0], [0.0,0.0,0.0,1.0]], 2.0, 2.0, 2.0),!).


test(change_with_create_to_odom) :-
	create_object_state_with_close(_, [[-83.0,12.0,-18.0],[0.0,0.0,0.0,1.0]], 'milk', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
	(rdf_equal(knowrob:'milk1',Name),get_object_infos(Name, '/odom_combined', milk, _, [[-83.0,12.0,-18.0], [0.0,0.0,0.0,1.0]], 2.0, 2.0, 2.0),!),
	create_object_state_with_close(_, [[-73.0,12.0,-18.0],[0.0,0.0,0.0,1.0]],[[-83.0,12.0,-18.0],[0.0,0.0,0.0,1.0]], 'milk', '/tree', 2.0, 2.0, 2.0, [_], _),
	(rdf_equal(knowrob:'milk1',Name),get_object_infos(Name, '/tree', milk, _, [[-73.0,12.0,-18.0], [0.0,0.0,0.0,1.0]], 2.0, 2.0, 2.0),!).


test(physical_parts_cake_spatula) :-
	create_object_state_with_close(_, [[13.0,12.0,11.0],[0.0,0.0,0.0,1.0]], 'cakeSpatula', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
	get_physical_parts(knowrob:'cakeSpatula1', _, knowrob:'HandleOfCakeSpatula1'),
	get_physical_parts(knowrob:'cakeSpatula1', _, knowrob:'SupportingPlaneOfCakeSpatula1'),
	get_tf_infos('SupportingPlaneOfCakeSpatula1', '/cakeSpatula1', [0.23,0.0,0.01],_),!.


test(physical_parts_cake_knife) :-
	create_object_state_with_close(_, [[130.0,120.0,110.0],[0.0,0.0,0.0,1.0]], 'cakeKnife', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
	get_physical_parts(knowrob:'cakeKnife1', _, knowrob:'BladeOfCakeKnife1'),
	get_physical_parts(knowrob:'cakeKnife1', _, knowrob:'HandleOfCakeKnife1' ),!.

test(get_infos) :-
	create_object_state_with_close(_, [[-5.0,-5.0,-5.0],[0.0,0.0,0.0,1.0]], 'dinnerPlateForCake', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
	get_info([[nameOfObject,dinnerPlateForCake1],radius], Ret),
	Ret=[[radius,'0.11']],
	get_info([[nameOfObject,dinnerPlateForCake1],angle], Ret2),
	Ret2=[[angle,'1.92']],!.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% HELPER FUNCT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


loop(0) :- create_object_state_with_close(_, [[1.0,1.0,1.0],[0.0,0.0,0.0,1.0]], 'cake', '/odom_combined', 2.0, 2.0, 2.0, [_], _).
loop(NumberOfSteps) :-
  (NumberOfSteps > 0),
  create_object_state_with_close(_, [[1.0,1.0,1.0],[0.0,0.0,0.0,1.0]], 'cake', '/odom_combined', 2.0, 2.0, 2.0, [_], _),
  New is NumberOfSteps - 1,
  loop(New).


:- end_tests(prolog_object_state).

