# Weather Station Front End

## Introduction

This Node/Angular-based web application allows for the viewing of weather patterns recorded by the Weather Station and stored into a database.

## Running the application

After cloning, navigate to this `ng-weather-station-frontend` directory, then run:
```
npm install
```

If angular-cli is not yet installed, run the command:
```
npm install -g @angular/cli
```

Create a file in the `ng-weather-station-frontend` directory named `.env` and specify your MongoDB information in the following format:
```
MONGO_HOST=Your_Host
MONGO_PORT=Your_Port_Default_27017
MONGO_DB=Your_DB_name
MONGO_COLL=Your_DB_Collection
```
Additionally, if you want the application to run on a port other than 8080, specify it in `.env`.
```
PORT=Your_port
```

Then, in the terminal, use the command:
```
npm start
```


## Defining categories

The interface allows for the separation of data based on their categories and units of measurement. The category may be any valid field string (e.g., `Speed`). It is possible to group data of similar categories but of different units based on the field names in the MongoDB. This subcategorization can done by appending an underscore and the unit to the end of the category (e.g., `Temperature_F`). The user will be allowed to select which unit they would like to be displayed.

Please attempt to keep these field names consistent.


