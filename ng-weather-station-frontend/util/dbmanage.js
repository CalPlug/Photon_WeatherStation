var dbClient = require('mongodb').MongoClient;
var uriMongo = `mongodb://${process.env.MONGO_HOST||'localhost'}:${process.env.MONGO_PORT||27017}/${process.env.MONGO_DB}`;

dbClient.connect(uriMongo, function(err, db){
    if (err){
        console.error('Could not connect to database.\n'+err);
        return;
    }
    console.log('Successfully connection to database');
    db.close();
});

//DB stuff
// These functions are placeholders for now.

var queryCategories = function(){
    return 'derp';
}

var queryDataByCategory = function(category, callback, hoursPast){
    if (typeof(hoursPast) === 'undefined') hoursPast = 24;
    
    dbClient.connect(uriMongo, function(err, db){
        if (err){callback(err, []);}
        
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
            callback
        );
    });
}

module.exports = {
    "queryCategories": queryCategories,
    "queryDataByCategory": queryDataByCategory
}