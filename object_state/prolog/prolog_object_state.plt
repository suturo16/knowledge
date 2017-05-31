
%% This file contains tests for the swrl module
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

:- begin_tests(object_state).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('swrl')).

:- owl_parser:owl_parse('package://object_state/owl/test_actions.owl').

:- rdf_db:rdf_register_prefix(suturo_actions, 'http://knowrob.org/kb/suturo_actions#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).

test_rule_id(Id, Descr) :-
  ( rdf_has(Descr, rdfs:label, literal(type(_,Id))) ;
    rdf_has(Descr, rdfs:label, literal(Id)) ),
  rdf_has(Descr, rdf:type, swrl:'Imp').

test(swrl_parse_rules) :-
  forall( rdf_has(Descr, rdf:type, swrl:'Imp'), (
    rdf_swrl_rule(Descr, Head :- Body),
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

%%
% Unit-Test for the cutting Action
test(cuttingtest) :-
  Cake='http://knowrob.org/kb/test_actions#cake1',
  Act='http://knowrob.org/kb/test_actions#cuttingAPieceOfFood1',
  \+ owl_has(Act, knowrob:outputsCreated, _),
  test_swrl_project('CuttingCake',[['cake',Cake],['act',Act]]),
  owl_has(Act, knowrob:outputsCreated, Piece),
  owl_individual_of(Piece, suturo_actions:'PieceOfCake').

  %\+ owl_individual_of(X, test_swrl:'Cake'),
  %\+ owl_individual_of(Y, test_swrl:'CuttingAPieceOfFood')
  %test_swrl_project('cuttingAPieceOfFood1',[['x',X]]),
  %owl_individual_of(X, test_swrl:'Person').

%test(swrl_Person1, [nondet]) :-
%  \+ swrl_holds(test_swrl:'Alex', rdf:type, test_swrl:'Person'),
%  rdf_swrl_load('Person'),
%  swrl_holds(test_swrl:'Alex', rdf:type, test_swrl:'Person').

%%% new rule -- needs to be debugged
owl_has(suturo_actions:'cuttingAPieceOfFood1'),
\+ owl_has(suturo_actions:'cuttingAPieceOfFood1', knowrob:outputsCreated, _),
rdf_swrl_load('CuttingCake'),
owl_has_(suturo_actions:'cuttingAPieceOfFood1', knowrob:outputsCreated, Piece)),
owl_individual_of(Piece, suturo_actions:'PieceOfCake').

:- end_tests(object_state).