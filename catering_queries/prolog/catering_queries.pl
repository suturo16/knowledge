/** <module> prolog_object_state

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

get_current_edibles(FoodClass) :-
      setof(A,get_current_edibles_help(FoodClass),_).

get_current_edibles_help(FoodClass) :-
      current_temporal_part(Obj,_),
      owl_has(Obj,rdf:type,FoodClass),
      owl_subclass_of(FoodClass, knowrob:'FoodOrDrink').

get_possible_recipe(Recipe) :-
      setof(_,
      (owl_subclass_of(Recipe, knowrob:'FoodOrDrink'),
      get_ingredients_as_list(Recipe,IngredientList),
      get_current_ingredients_as_list(CurrIngredientList),
      subset(IngredientList,CurrIngredientList)),_).

get_ingredients_as_list(Food,Ingredientlist) :-
      setof(Ingredient,get_ingredients(Food,Ingredient),Ingredientlist).

get_current_ingredients_as_list(Ingredientlist) :-
      setof(Ingredient,get_current_ingredients(Ingredient),Ingredientlist).

get_ingredients(Food,Ingredient) :-
      rdf_has(Food,rdfs:subClassOf,Restr),
      rdf_has(Restr,owl:'onProperty',knowrob:hasIngredient),
      rdf_has(Restr,owl:'someValuesFrom',Ingredient).

get_current_ingredients(Ingredient) :-
      setof(A,get_current_ingredients_help(Ingredient),_).

get_current_ingredients_help(IngredientClass) :-
      current_temporal_part(Obj,_),
      owl_has(Obj,rdf:type,IngredientClass),
      owl_subclass_of(IngredientClass, knowrob:'FoodIngredientOnly').

