/** <module> suturo_restaurant_organization

@author Sascha Jongebloed 
@license BSD
*/

% defining functions
:- module(suturo_restaurant_organization,
    [
      getCustomerInfos/3,
      getOpenOrdersOf/3,
      getOpenOrdersWithCustomerInfos/5,
      getFreeTable/1      
    ]).

%getCustomerInfos(+CustomerID,-Name,-Place)
getCustomerInfos(CustomerID,Name,Place) :-
      getOpenOrdersWithCustomerInfos(CustomerID,Name,Place,_,_) .


%getOpenOrdersOf(+customerID,-Item,-Amount)
getOpenOrdersOf(CustomerID,Item,Amount) :-
      getOpenOrdersWithCustomerInfos(CustomerID,_,_,Item,Amount).


%getOpenOrdersWithCustomerInfos(+CustomerID,-Name,-Place,-Good,-Amount)
getOpenOrdersWithCustomerInfos(CustomerID,Name,Place,Item,Amount) :-
      Name='michael',
      Place='table1',
      Item='cake',
      Amount=1.

%getFreeTables(-NameOfFreeTable)
getFreeTable(NameOfFreeTable) :-
      NameOfFreeTables='table2'.