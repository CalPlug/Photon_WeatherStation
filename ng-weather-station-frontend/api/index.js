var app = require('express');
const router = app.Router({ mergeParams: true });

var db = require('../util/dbmanage');

// GET /api/categories
// Get the categories, e.g., Temperature_F, Temperature_C,...
router.get('/categories', function(req, res){
    
    //Initial premise testing
    res.status(200).json(
        {
            'status':"OK", 
            'categories':['Temperature_F', 'Temperature_C']
        }
    );
});

// GET /api/data/<category>
//Get the recorded data for a category
router.get('/data/:category', function(req, res){

    //Initial premise testing
    res.status(200).json(
        {
            'status':"OK", 
            'category':req.params.category,
            'data':['25', '70']
        }
    );
});

module.exports = router;