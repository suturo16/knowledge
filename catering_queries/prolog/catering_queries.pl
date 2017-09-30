/** <module> prolog_object_state

  Copyright (C) 2017 Sascha Jongebloed
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(catering_queries,
    [
      get_current_edibles/1,
      get_current_edibles_help/1,
      get_possible_recipe/1,
      get_ingredients_as_list/2,
      get_ingredients/2,
      get_current_ingredients_as_list/1,
      get_current_ingredients_help/1,
      get_current_ingredients/1
    ]).

%% get_current_edibles(?FoodClass)
%
% Returns the classes of edibles currently in the world state
%
% @FoodClass Classes of edibles
%
get_current_edibles(FoodClass) :-
      setof(A,get_current_edibles_help(FoodClass),_).

get_current_edibles_help(FoodClass) :-
      current_temporal_part(Obj,_),
      owl_has(Obj,rdf:type,FoodClass),
      owl_subclass_of(FoodClass, knowrob:'FoodOrDrink').

%% get_possible_recipe(?Recipe)
%
% Returns all recipes for which we have enough ingredients in the world state
%
% @Recipe Recipes
%
get_possible_recipe(Recipe) :-
      setof(_,
      (owl_subclass_of(Recipe, knowrob:'FoodOrDrink'),
      get_ingredients_as_list(Recipe,IngredientList),
      get_current_ingredients_as_list(CurrIngredientList),
      subset(IngredientList,CurrIngredientList)),_).

%% get_ingredients_as_list(?Food,-Ingredientlist) 
%
% Returns FoodOrDrink Object with their ingredients
%
% @Food FoodOrDrink
% @Ingredientlist List of needed Ingredients
%
get_ingredients_as_list(Food,Ingredientlist) :-
      setof(Ingredient,get_ingredients(Food,Ingredient),Ingredientlist).

get_current_ingredients_as_list(Ingredientlist) :-
      setof(Ingredient,get_current_ingredients(Ingredient),Ingredientlist).

%% get_ingredients_as_list(?Food,-Ingredient) 
%
% Returns the ingredients of a FoodOrDrink Object
%
% @Food FoodOrDrink
% @Ingredientlist List of needed Ingredients
%
get_ingredients(Food,Ingredient) :-
      rdf_has(Food,rdfs:subClassOf,Restr),
      rdf_has(Restr,owl:'onProperty',knowrob:hasIngredient),
      rdf_has(Restr,owl:'someValuesFrom',Ingredient).

%% get_current_ingredients(Ingredient)
%
% Returns the ingredients in the world state
%
% @Ingredient Ingredient in the world state.
%
get_current_ingredients(Ingredient) :-
      setof(A,get_current_ingredients_help(Ingredient),_).

get_current_ingredients_help(IngredientClass) :-
      current_temporal_part(Obj,_),
      owl_has(Obj,rdf:type,IngredientClass),
      owl_subclass_of(IngredientClass, knowrob:'FoodIngredientOnly').

