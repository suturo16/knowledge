
%% This file contains tests for the catering_queries module
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

:- begin_tests(dialog_management).

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(catering_queries).

:- use_module(library('catering_queries')).
:- use_module(library('suturo_restaurant_organization')).
:- use_module(library('dialog_management')).

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_common/owl/knowrob_common.owl').

:- owl_parse('package://object_state/owl/suturo_objects.owl').
:- owl_parse('package://catering_queries/owl/suturo_recipes.owl').
:- owl_parse('package://catering_queries/owl/suturo_dialog.owl').

:- use_module(library('knowrob_owl')).

%%%%%%%%%%%%%%%%%%%%%%%%%TESTS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

test(normal_use_case) :-
	assert_dialog_element('{guestId:arthur1,query:{type:setCake,amount:5,guestName:arthur}}'),
	assert_dialog_element('{guestId:arthur1,query:{type:setLocation,tableId:table1}}'),
	get_open_orders_with_customer_infos('arthur1','arthur',knowrob:'table1',cake,5,0),!,
	assert_dialog_element('{guestId:arthur1,query:{type:increaseCake,amount:4}}'),
	get_open_orders_with_customer_infos('arthur1','arthur',knowrob:'table1',cake,9,0).

:- end_tests(dialog_management).
