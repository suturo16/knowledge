
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
:- rdf_db:rdf_register_prefix(test_actions, 'http://knowrob.org/kb/test_actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).

%%
% LSa
% Unit-Test for the cutting action effect.
test(swrl_CuttingCake, [nondet]) :-
  \+ owl_has(suturo_actions:'cuttingAPieceOfFood1', knowrob:outputsCreated, _),
  rdf_swrl_project('CuttingCake'),
  owl_has(suturo_actions:'cuttingAPieceOfFood1', knowrob:outputsCreated, Piece),
  owl_individual_of(Piece, suturo_actions:'PieceOfCake').

:- end_tests(object_state). 