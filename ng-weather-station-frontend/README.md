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
Additionally, if you want the application to run on a different port than 8080, specify it in `.env`.
```
PORT=Your_port
```

Then, in the terminal, use the command:
```
npm start
```
