var app = require('express');
const router = app.Router({ mergeParams: true });

var db = require('../util/dbmanage');

// GET /api/categories
// Get the categories, e.g., Temperature_F, Temperature_C,...
router.get('/categories', function(req, res){
    
    db.queryCategories(function(err, result){
        //Initial premise testing
        res.status(200).json(
            {
                'status':"OK", 
                'categories':result
            }
        );
    });

    
});

// GET /api/data/<category>
//Get the recorded data for a category
router.get('/data/:category', function(req, res){

    db.queryDataByCategory(req.params.category, function(err, result){
        if (err)console.error(err);
        //Initial premise testing
        res.status(200).json(
            {
                'status':"OK", 
                'category':req.params.category,
                'data':result
            }
        );
    });

    
});

module.exports = router;