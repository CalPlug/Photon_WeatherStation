import { Component, OnInit } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Http, Response, Headers, RequestOptions } from '@angular/http';
import { Observable } from 'rxjs/Observable';

import { Category } from '../category';

@Component({
    selector: 'app-datapage',
    templateUrl: './datapage.component.html',
    styleUrls: ['./datapage.component.css']
})
export class DatapageComponent implements OnInit {

    private apiUrl = 'api/';

    categories:Category[] = [];
    cateogriesObj= {};
    categoryData = {};

    constructor( private http: HttpClient ) { }

    ngOnInit() {
        this.getCategories();
    }

    getCategories() {
        this.http.get(this.apiUrl+'categories').subscribe(data => {
            var catData:string[] = data['categories'];
            var lenCats:number = catData.length;
            for (var i:number = 0; i<lenCats; ++i){
                var cat:string = catData[i];

                var parent:string = cat.split('_',1)[0];

                if (parent) {
                    
                    if (!(parent in this.cateogriesObj))
                        this.cateogriesObj[parent] = new Category(parent);

                    if (cat != parent){
                        var index = cat.indexOf('_');
                        var child = cat.substring(index+1);
                        this.cateogriesObj[parent].children.push(child);
                    }
                }
            }
            this.categories = Object.values(this.cateogriesObj);
            // console.log(this.categories);
        });
    }

    getCategoryData(category){
        this.http.get(this.apiUrl+'/data/'+category).subscribe(data => {
            var catData:string[] = data['data'];
            this.categoryData[category] = catData;
        });
    }



}
