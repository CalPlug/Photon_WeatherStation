var mongodb = require('mongodb');
var dbClient = mongodb.MongoClient;

const uri = `mongodb://${process.env.MONGO_HOST || "localhost"}:${process.env.MONGO_PORT || 27017}/Weather_Station`;


function dbConnect(){
    dbClient.connect(uri, function(err, db) {
        if (err) { 
            console.log(err);
            dbConnected = false;
            return;
        }
        console.log(`Connected to MongoDB at ${url}`);
        // db.close();
    });
}

process.on('beforeExit', function () { 
    dbClient.close();
});

process.on('SIGINT', function () { 
    dbClient.close();
});

var queryCategories = function(){
    return 'derp';
}

var queryDataByCategory = function(category){
    return 'derp';
}

module.exports = {
    "queryCategories": queryCategories,
    "queryDataByCategory": queryDataByCategory
}