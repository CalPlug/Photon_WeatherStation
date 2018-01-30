var dbClient = require('mongodb').MongoClient;
var uriMongo = `mongodb://${process.env.MONGO_HOST||'localhost'}:${process.env.MONGO_PORT||27017}/${process.env.MONGO_DB}`;

dbClient.connect(uriMongo, function(err, db){
    if (err){
        console.error('Could not connect to database.\n'+err);
        return;
    }
    console.log('Successful connection to database');
    db.close();
});


// Coded under the assumption that all documents will be consistent.
// Based on the Python code, they will be!
var queryCategories = function(callback){

    dbClient.connect(uriMongo, function(err, db){
        if (err){callback(err, []); db.close(); return;}

        db.db(process.env.MONGO_DB).collection(process.env.MONGO_COLL).findOne({}, (
            function (err, result){
                if (err){callback(err, []); db.close(); return;}

                var currKeys = Object.keys(result);
                var index = currKeys.indexOf('utcTime')
                if (index > -1){
                    currKeys.splice(index, 1);
                }

                callback(null, currKeys);
                db.close();
            }
        ));
    });
}

var queryDataByCategory = function(category, callback, hoursPast){
    if (typeof(hoursPast) === 'undefined') hoursPast = 24;

    dbClient.connect(uriMongo, function(err, db){
        if (err){callback(err, []); db.close(); return;}
        
        var date = new Date(new Date().getTime() - (hoursPast * 60 * 60 * 1000));
        db.db(process.env.MONGO_DB).collection(process.env.MONGO_COLL).find(
            {
                utcTime:{
                    "$gte": date
                }
            }
        ).project(
            { [category]:1, utcTime:1, _id:0 }
        ).toArray(
            function(err, result){
                callback(err, result);
                db.close();
            }
        );
    });
}

module.exports = {
    "queryCategories": queryCategories,
    "queryDataByCategory": queryDataByCategory
}